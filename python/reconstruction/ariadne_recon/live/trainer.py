"""The incremental trainer (RECON_6 §2; D190-D193, D195): one keyframe at a time, only the active submap.

Per keyframe of agent k:
  1. assign it to k's active submap (SubmapManager); a new submap starts with no Gaussians
  2. render the submap at the keyframe (no grad) -> alpha; seed new Gaussians where alpha < 0.5 (seeding.py), append
     them to the parameters and to Adam's state (old rows keep their moments: checked on a T4 by bench_kernel.py)
  3. optimisation steps: >= 40 % on the new keyframe, the rest on uniformly drawn keyframes of the same submap.
     Gaussian-SLAM's loss (arXiv 2312.10070 eq. 10-12, D198): L = l_color (0.8 L1 + 0.2 (1 - SSIM)) + l_depth
     (metric L1, metres) + l_reg (isotropy: each Gaussian's |s - mean of its own s|), all weights 1, over pixels
     where image and depth are defined (§19.3's L_photo + L_depth; L_light and §19.4 are the batch path's).
     Gaussians with opacity < 0.5 are pruned at the middle and the end of a keyframe's iterations (TUM value)
  4. lifecycle (arch §18.4, D191): a Gaussian visible in the keyframe whose camera-frame depth agrees with the
     keyframe's stereo depth within 3 sigma at its centre pixel gains one count; at `stable_count` (15) it is STABLE
     and frozen: its rows are restored after every Adam step and their moments zeroed (zeroing the gradient alone
     leaves Adam's exp_avg moving them)
  5. exposure (arch §18.7, D192): a per-keyframe affine on the rendered image, identity, trainable only once the
     submap's own step counter has passed 5000

Iterations per keyframe: fixed (batch/gate path: Gaussian-SLAM's TUM 100 for a submap's first keyframe and 100 for
the rest), or `budget_ms` (live): iterations = budget / the measured median ms per iteration of this submap so far
(derived from measurement, D190; at least `min_iters`).

The training image is the albedo proxy (1 channel, D195) unless the run is the RGB ablation (3 channels).
GPU only (gsplat rasterizes in CUDA); the CPU parts live in submaps.py / seeding.py / export.py.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .seeding import SeedParams, seed
from .submaps import SubmapManager

SH_C0 = 0.28209479177387814


@dataclass(frozen=True)
class TrainParams:
    iters: int = 0                # > 0: fixed iterations per keyframe (batch path; Gaussian-SLAM TUM: 100)
    iters_first: int = 0          # > 0: a submap's first keyframe (Gaussian-SLAM TUM: 100); 0 = iters
    budget_ms: float = 0.0        # > 0: live, iterations from the measured ms/iteration
    max_live_iters: int = 64      # hard work cap even when timing history is unusually small
    min_iters: int = 3
    new_kf_fraction: float = 0.4  # Gaussian-SLAM: >= 40 % of the iterations on the new keyframe
    lambda_dssim: float = 0.2     # Gaussian-SLAM eq. 10 (= Kerbl 2023)
    lambda_color: float = 1.0     # Gaussian-SLAM: lambda_color = lambda_depth = lambda_reg = 1
    lambda_depth: float = 1.0
    lambda_reg: float = 1.0
    depth_loss: str = "metric_l1" # metric_l1 (Gaussian-SLAM, MonoGS) | none (diagnostic only)
    stable_count: int = 15        # D191: RTG delta_eta 100 frames at 30 Hz -> 15 keyframes at 4.4 KF/s
    prune_opacity: float = 0.5    # Gaussian-SLAM o_thre (TUM / real data; 0.1 on synthetic Replica)
    init_opacity: float = 0.5     # Gaussian-SLAM
    exposure_after_steps: int = 5000   # arch §18.7 / D192
    max_keyframes: int = 30       # RECON_6: arch 20-40
    lr_means: float = 1.6e-4      # gsplat simple_trainer defaults (means lr scaled by scene scale 1)
    lr_scales: float = 5e-3
    lr_quats: float = 1e-3
    lr_opacities: float = 5e-2
    lr_colors: float = 2.5e-3
    seed: int = 0
    diagnostics: bool = False     # log training-view PSNR before/after a keyframe's iterations and check freezing


def ssim(torch, a, b):
    """SSIM of [1, H, W, C] images, 11x11 sigma 1.5 Gaussian window (the 3DGS reference loss)."""
    F = torch.nn.functional
    c = a.shape[-1]
    x, y = a.permute(0, 3, 1, 2), b.permute(0, 3, 1, 2)
    g = torch.exp(-((torch.arange(11, device=a.device) - 5.0) ** 2) / (2 * 1.5 ** 2))
    w = ((g[:, None] * g[None, :]) / g.sum() ** 2).expand(c, 1, 11, 11).contiguous()
    blur = lambda t: F.conv2d(t, w, padding=5, groups=c)
    mx, my = blur(x), blur(y)
    sxx, syy, sxy = blur(x * x) - mx * mx, blur(y * y) - my * my, blur(x * y) - mx * my
    c1, c2 = 0.01 ** 2, 0.03 ** 2
    return ((2 * mx * my + c1) * (2 * sxy + c2)) / ((mx * mx + my * my + c1) * (sxx + syy + c2))


def freeze_rows(opt, params: dict, mask, saved: dict) -> None:
    """Undo an optimiser step for the rows in `mask` and zero their Adam moments, so frozen rows stay bit-exact."""
    for g in opt.param_groups:
        p = g["params"][0]
        name = g["name"]
        p.data[mask] = saved[name]
        st = opt.state.get(p)
        if st:
            for k in ("exp_avg", "exp_avg_sq"):
                if k in st:
                    st[k][mask] = 0


class _SubmapState:
    """The GPU tensors of one submap (anchor frame) and its optimiser."""

    def __init__(self, torch, dev, channels, p: TrainParams, sid: int):
        self.t, self.dev, self.p, self.sid = torch, dev, p, sid
        z = lambda *s: torch.zeros(*s, device=dev)
        self.params = {"means": torch.nn.Parameter(z(0, 3)), "scales": torch.nn.Parameter(z(0, 3)),
                       "quats": torch.nn.Parameter(z(0, 4)), "opacities": torch.nn.Parameter(z(0)),
                       "colors": torch.nn.Parameter(z(0, channels))}
        lr = {"means": p.lr_means, "scales": p.lr_scales, "quats": p.lr_quats, "opacities": p.lr_opacities,
              "colors": p.lr_colors}
        self.opt = torch.optim.Adam([{"params": [v], "lr": lr[k], "name": k} for k, v in self.params.items()],
                                    eps=1e-15)
        self.count = z(0)                      # lifecycle confidence (keyframe updates)
        self.stable = torch.zeros(0, dtype=torch.bool, device=dev)
        self.steps = 0
        self.ms_iter: list[float] = []
        self.exposure: dict[int, object] = {}  # kf -> (C+1) x C affine, identity, trainable after the delay
        self.exp_opt = None
        self.frozen_ok = True

    def __len__(self):
        return int(self.params["means"].shape[0])

    def _rebuild(self, new: dict | None, keep=None):
        """Grow (new rows) or shrink (keep mask) every parameter and carry Adam's moments row for row."""
        torch = self.t
        for g in self.opt.param_groups:
            old = g["params"][0]
            name = g["name"]
            data = old.detach()
            st = self.opt.state.pop(old, None)
            if keep is not None:
                data = data[keep]
            if new is not None:
                data = torch.cat([data, new[name]], 0)
            p = torch.nn.Parameter(data)
            g["params"][0] = p
            if st is not None:
                for k in ("exp_avg", "exp_avg_sq"):
                    m = st[k] if keep is None else st[k][keep]
                    if new is not None:
                        m = torch.cat([m, torch.zeros_like(new[name])], 0)
                    st[k] = m
                self.opt.state[p] = st
            self.params[name] = p

    def append(self, means, scales, colors, init_opacity):
        torch = self.t
        n = len(means)
        if n == 0:
            return
        f = lambda a: torch.as_tensor(np.asarray(a), dtype=torch.float32, device=self.dev)
        q = torch.zeros(n, 4, device=self.dev)
        q[:, 0] = 1.0
        new = {"means": f(means), "scales": torch.log(f(scales))[:, None].repeat(1, 3), "quats": q,
               "opacities": torch.full((n,), float(np.log(init_opacity / (1 - init_opacity))), device=self.dev),
               "colors": f(colors)}
        self._rebuild(new)
        self.count = torch.cat([self.count, torch.zeros(n, device=self.dev)])
        self.stable = torch.cat([self.stable, torch.zeros(n, dtype=torch.bool, device=self.dev)])

    def prune(self, threshold):
        torch = self.t
        keep = torch.sigmoid(self.params["opacities"].detach()) >= threshold
        if bool(keep.all()):
            return 0
        n = int((~keep).sum())
        self._rebuild(None, keep)
        self.count, self.stable = self.count[keep], self.stable[keep]
        return n


class LiveTrainer:
    def __init__(self, channels: int, p: TrainParams, seed_params: SeedParams = SeedParams(), device: str = "cuda"):
        import torch
        from gsplat import rasterization
        self.torch, self.raster = torch, rasterization
        self.dev = torch.device(device)
        self.p, self.sp, self.channels = p, seed_params, channels
        self.manager = SubmapManager(p.max_keyframes)
        self.state: dict[int, _SubmapState] = {}
        # Keyframe IDs are agent-local. A submap ID makes the training-data key globally unique.
        self.frames: dict[tuple[int, int], dict] = {}  # (submap_id, kf) -> tensors; active submaps only
        self.rng = np.random.default_rng(p.seed)
        self.log: list[dict] = []

    # --- rendering -------------------------------------------------------------------------------------------------
    def _render(self, st: _SubmapState, kf: int, grad: bool):
        torch = self.torch
        fr = self._frame(st, kf)
        P = st.params
        ctx = torch.enable_grad() if grad else torch.no_grad()
        with ctx:
            img, alpha, meta = self.raster(P["means"], torch.nn.functional.normalize(P["quats"], dim=-1),
                                           torch.exp(P["scales"]), torch.sigmoid(P["opacities"]), P["colors"],
                                           fr["viewmat"][None], fr["K"][None], fr["w"], fr["h"], sh_degree=None,
                                           packed=False, render_mode="RGB+ED", near_plane=0.01, far_plane=100.0)
        return img[0], alpha[0, ..., 0], meta

    # --- one keyframe ----------------------------------------------------------------------------------------------
    def add_keyframe(self, agent: int, kf: int, T_O_C: np.ndarray, pose_version: int, K: np.ndarray,
                     image: np.ndarray, Z: np.ndarray, sigma: np.ndarray) -> dict:
        torch = self.torch
        t0 = time.perf_counter()
        sm, opened = self.manager.add_keyframe(agent, kf, T_O_C, pose_version)
        if opened:
            self.state[sm.sid] = _SubmapState(torch, self.dev, self.channels, self.p, sm.sid)
            for other in [s for s in self.manager.submaps if s.agent == agent and s.closed and s.sid in self.state]:
                self._close(other.sid)
        st = self.state[sm.sid]
        h, w = Z.shape
        T_A_C = sm.T_anchor_C[kf]
        img = np.asarray(image, np.float32).reshape(h, w, -1)
        f = lambda a: torch.as_tensor(np.ascontiguousarray(a), dtype=torch.float32, device=self.dev)
        valid = np.isfinite(img).all(axis=2) & np.isfinite(Z) & np.isfinite(sigma) & (sigma <= self.sp.max_sigma_m)
        frame_key = (sm.sid, kf)
        self.frames[frame_key] = {"viewmat": f(np.linalg.inv(T_A_C)), "K": f(K), "w": w, "h": h, "sid": sm.sid,
                                  "img": f(np.nan_to_num(img)), "Z": f(np.nan_to_num(Z)), "S": f(np.nan_to_num(sigma, nan=1.0)),
                                  "mask": torch.as_tensor(valid, device=self.dev)}
        alpha = None
        if len(st):
            _, a, _ = self._render(st, kf, grad=False)
            alpha = a.detach().cpu().numpy()
        existing = st.params["means"].detach().cpu().numpy() if len(st) else None
        s = seed(Z, sigma, img, K, T_A_C, alpha, existing, self.sp, self.rng, first=opened)
        st.append(s.means, s.scales, s.colors, self.p.init_opacity)
        psnr_init = self._psnr(st, kf) if self.p.diagnostics else None
        n_iter = self._iterations(st, opened)
        n_new = max(1, round(self.p.new_kf_fraction * n_iter))
        losses, terms, pruned = [], None, 0
        opt_start = time.perf_counter()
        planned_iters = n_iter
        for i in range(n_iter):
            loss, terms = self._step(st, kf if i < n_new else int(self.rng.choice(sm.keyframes)))
            losses.append(loss)
            if i == n_iter // 2 or i == n_iter - 1:   # Gaussian-SLAM: prune at the middle and the end
                pruned += st.prune(self.p.prune_opacity)
            if (self.p.iters == 0 and self.p.budget_ms > 0 and i + 1 >= self.p.min_iters
                    and (time.perf_counter() - opt_start) * 1000 >= self.p.budget_ms):
                if i != planned_iters // 2 and i != planned_iters - 1:
                    pruned += st.prune(self.p.prune_opacity)
                break
        n_iter = len(losses)
        optimization_ms = 1000 * (time.perf_counter() - opt_start)
        newly_stable = self._lifecycle(st, kf)
        psnr_after = self._psnr(st, kf) if self.p.diagnostics else None
        torch.cuda.synchronize()
        rec = {"agent": agent, "kf": kf, "sid": sm.sid, "opened": opened, "seeded": len(s.means), "gaussians": len(st),
               "iters": n_iter, "planned_iters": planned_iters, "optimization_ms": optimization_ms,
               "budget_ms": self.p.budget_ms, "loss_first": losses[0] if losses else None, "loss_last": losses[-1] if losses else None,
               "terms_last": terms, "psnr_train_init": psnr_init, "psnr_train_after": psnr_after,
               "frozen_bitwise": st.frozen_ok if self.p.diagnostics else None,
               "stable": int(st.stable.sum()), "newly_stable": newly_stable, "pruned": pruned, "steps": st.steps,
               "exposure_affine": st.steps >= self.p.exposure_after_steps,
               "ms_iter_median": float(np.median(st.ms_iter)) if st.ms_iter else None,
               "ms_total": 1e3 * (time.perf_counter() - t0)}
        self.log.append(rec)
        return rec

    def _psnr(self, st: _SubmapState, kf: int):
        """Masked PSNR of the submap's render at a keyframe against its training image (diagnostics only)."""
        torch = self.torch
        if not len(st):
            return None
        fr = self._frame(st, kf)
        img, _, _ = self._render(st, kf, grad=False)
        m = fr["mask"][..., None].float()
        mse = float((((img[..., : self.channels].clamp(0, 1) - fr["img"]) ** 2) * m).sum() / (m.sum() * self.channels))
        return round(10 * float(np.log10(1.0 / max(mse, 1e-12))), 2)

    def _iterations(self, st: _SubmapState, first: bool = False) -> int:
        if self.p.iters > 0:
            return self.p.iters_first if first and self.p.iters_first > 0 else self.p.iters
        if self.p.budget_ms > 0:
            if not st.ms_iter:
                return self.p.min_iters
            median = float(np.median(st.ms_iter[-50:]))
            if not np.isfinite(median) or median <= 0:
                return self.p.min_iters
            return min(self.p.max_live_iters, max(self.p.min_iters, int(self.p.budget_ms / median)))
        raise ValueError("TrainParams: set iters (batch) or budget_ms (live)")

    def _exposure(self, st: _SubmapState, kf: int):
        torch = self.torch
        if kf not in st.exposure:
            c = self.channels
            A = torch.zeros(c + 1, c, device=self.dev)
            A[:c, :c] = torch.eye(c, device=self.dev)
            st.exposure[kf] = torch.nn.Parameter(A)
            if st.exp_opt is None:
                st.exp_opt = torch.optim.Adam([st.exposure[kf]], lr=1e-3)
            else:
                st.exp_opt.add_param_group({"params": [st.exposure[kf]], "lr": 1e-3})
        return st.exposure[kf]

    def _step(self, st: _SubmapState, kf: int) -> tuple[float, dict]:
        torch = self.torch
        t0 = time.perf_counter()
        fr = self._frame(st, kf)
        img, _, _ = self._render(st, kf, grad=True)
        rgb, depth = img[..., : self.channels], img[..., self.channels:]
        use_exp = st.steps >= self.p.exposure_after_steps
        if use_exp:   # D192: identity until the submap's own step counter passes the delay
            A = self._exposure(st, kf)
            rgb = rgb @ A[: self.channels] + A[self.channels]
        m = fr["mask"][..., None].float()
        n = m.sum().clamp(min=1.0)
        l1 = ((rgb - fr["img"]).abs() * m).sum() / (n * self.channels)
        dssim = ((1.0 - ssim(torch, rgb[None], fr["img"][None])) * m.permute(2, 0, 1)[None]).sum() / (n * self.channels)
        l_color = (1 - self.p.lambda_dssim) * l1 + self.p.lambda_dssim * dssim
        ld = ((depth[..., 0] - fr["Z"]).abs() * fr["mask"]).sum() / n if self.p.depth_loss == "metric_l1" \
            else torch.zeros((), device=self.dev)
        sc = torch.exp(st.params["scales"])
        l_reg = (sc - sc.mean(dim=1, keepdim=True)).abs().sum(dim=1).mean()
        loss = self.p.lambda_color * l_color + self.p.lambda_depth * ld + self.p.lambda_reg * l_reg
        st.opt.zero_grad(set_to_none=True)
        if st.exp_opt is not None:
            st.exp_opt.zero_grad(set_to_none=True)
        loss.backward()
        frozen = st.stable if bool(st.stable.any()) else None
        saved = {k: v.detach()[frozen].clone() for k, v in st.params.items()} if frozen is not None else None
        st.opt.step()
        if frozen is not None:   # STABLE Gaussians are frozen (arch §18.4): restore the rows, clear their moments
            freeze_rows(st.opt, st.params, frozen, saved)
            if self.p.diagnostics:
                st.frozen_ok = st.frozen_ok and all(torch.equal(st.params[k].detach()[frozen], saved[k]) for k in saved)
        if use_exp:
            st.exp_opt.step()
        st.steps += 1
        torch.cuda.synchronize()
        st.ms_iter.append(1e3 * (time.perf_counter() - t0))
        return float(loss.detach()), {"color": float(l_color.detach()), "depth": float(ld.detach()),
                                      "reg": float(l_reg.detach())}

    def _lifecycle(self, st: _SubmapState, kf: int) -> int:
        """Count consistent observations: visible in the keyframe and the Gaussian's camera-frame depth within 3 sigma
        of the stereo depth at its centre pixel (D191)."""
        torch = self.torch
        if not len(st):
            return 0
        fr = self._frame(st, kf)
        with torch.no_grad():
            V = fr["viewmat"]
            X = st.params["means"] @ V[:3, :3].T + V[:3, 3]
            z = X[:, 2]
            K = fr["K"]
            u = torch.round(K[0, 0] * X[:, 0] / z.clamp(min=1e-6) + K[0, 2]).long()
            v = torch.round(K[1, 1] * X[:, 1] / z.clamp(min=1e-6) + K[1, 2]).long()
            inside = (z > 0.01) & (u >= 0) & (u < fr["w"]) & (v >= 0) & (v < fr["h"])
            ui, vi = u.clamp(0, fr["w"] - 1), v.clamp(0, fr["h"] - 1)
            ok = inside & fr["mask"][vi, ui] & ((z - fr["Z"][vi, ui]).abs() <= 3 * fr["S"][vi, ui])
            st.count += ok.float()
            before = int(st.stable.sum())
            st.stable |= st.count >= self.p.stable_count
            return int(st.stable.sum()) - before

    def _close(self, sid: int):
        """A closed submap is frozen: drop its frames' training tensors (bounded memory) but keep its Gaussians."""
        for key in [key for key in self.frames if key[0] == sid]:
            del self.frames[key]
        st = self.state[sid]
        st.opt.state.clear()

    def _frame(self, st: _SubmapState, kf: int) -> dict:
        """Return a frame only from the owning submap; kf alone is not unique across agents."""
        return self.frames[(st.sid, kf)]

    # --- export ----------------------------------------------------------------------------------------------------
    def params_numpy(self, sid: int) -> dict:
        st = self.state[sid]
        P = {k: v.detach().cpu().numpy() for k, v in st.params.items()}
        c = np.clip(P["colors"], 0, 1)
        rgb = np.repeat(c, 3, axis=1) if c.shape[1] == 1 else c[:, :3]
        return {"means": P["means"], "scales": P["scales"], "quats": P["quats"], "opacities": P["opacities"],
                "sh0": ((rgb - 0.5) / SH_C0)[:, None, :].astype(np.float32)}
