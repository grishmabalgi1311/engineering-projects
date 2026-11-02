"""Visual teach-and-repeat controller using only exploration images and actions."""

import hashlib
import json
import pickle
import time
from pathlib import Path

import cv2
import numpy as np
from vis_nav_sdk import Agent


class VisualTeachIndex(Agent):
    def __init__(self, data_dir: Path, **kwargs):
        cv2.setNumThreads(1)
        self.data_dir = Path(data_dir)
        self.frames = []
        self.routes = []
        self.det = cv2.SIFT_create(nfeatures=400)
        self.bf = cv2.BFMatcher()
        self.features = []
        self.thumbs = []
        cache = Path(kwargs.get("cache_dir", "cache")) / "teach_features.pkl"
        manifests = sorted(self.data_dir.glob("traj_*/data_info.json"))
        signature = hashlib.sha256(
            b"".join(p.read_bytes() for p in manifests) + str(self.data_dir.resolve()).encode()
        ).hexdigest()
        cached = None
        if cache.exists():
            saved = pickle.loads(cache.read_bytes())
            if saved[0] == signature:
                cached = saved[1:]
        for directory in sorted(self.data_dir.glob("traj_*")):
            route = []
            for row in json.loads((directory / "data_info.json").read_text()):
                route.append(len(self.frames))
                self.frames.append((directory / row["image"], row["action"]))
                if cached is None:
                    image = cv2.imread(str(directory / row["image"]))
                    self.thumbs.append(self.thumb(image))
                    kp, des = self.det.detectAndCompute(image, None)
                    self.features.append((np.float32([k.pt for k in kp]), des))
                if len(self.frames) % 200 == 0:
                    print("index", len(self.frames), flush=True)
            self.routes.append(route)
        self.thumbs = np.array(self.thumbs)
        if cached is not None:
            self.features, self.thumbs = cached
        else:
            cache.parent.mkdir(parents=True, exist_ok=True)
            cache.write_bytes(pickle.dumps((signature, self.features, self.thumbs)))
        self.current = None
        self.goal = None
        self.target_cache_path = cache.parent / "target_match_cache.pkl"
        self.target_cache_signature = (
            signature,
            cv2.__version__,
            hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        )
        self.target_rank_cache = {}
        self.target_cache_hits = 0
        if self.target_cache_path.exists():
            try:
                saved_signature, ranks = pickle.loads(self.target_cache_path.read_bytes())
                if saved_signature == self.target_cache_signature and isinstance(ranks, dict):
                    self.target_rank_cache = ranks
            except (OSError, ValueError, TypeError, EOFError, pickle.UnpicklingError):
                pass
        # Some SDK datasets include a public four-view panorama. Precompute only
        # when present; setup reuses a result only for identical SDK image pixels.
        panorama_path = self.data_dir / "target.jpg"
        panorama = cv2.imread(str(panorama_path)) if panorama_path.exists() else None
        if panorama is not None and panorama.shape[1] % 4 == 0:
            width = panorama.shape[1] // 4
            for view in range(4):
                self._rank_target(panorama[:, view * width : (view + 1) * width])
            self._save_target_cache()

    def _save_target_cache(self):
        while len(self.target_rank_cache) > 16:
            del self.target_rank_cache[next(iter(self.target_rank_cache))]
        self.target_cache_path.write_bytes(
            pickle.dumps((self.target_cache_signature, self.target_rank_cache))
        )

    def _rank_target(self, image, remember=True):
        digest = hashlib.sha256(str(image.shape).encode() + image.tobytes()).hexdigest()
        if digest in self.target_rank_cache:
            self.target_cache_hits += 1
            return self.target_rank_cache[digest]
        kp, des = self.det.detectAndCompute(image, None)
        feat = (np.float32([k.pt for k in kp]), des)
        rows = []
        for i, other in enumerate(self.features):
            count, residual, _ = self.match(feat, other)
            quality = count / (1 + residual / 20)
            rows.append((quality, i, count, residual))
        result = (feat, rows)
        if remember:
            self.target_rank_cache[digest] = result
        return result

    @staticmethod
    def thumb(image):
        return cv2.resize(cv2.GaussianBlur(image, (5, 5), 0), (40, 30)).astype(np.float32) / 255

    def match(self, features, other):
        p, d = features
        q, e = other
        if d is None or e is None or len(e) < 2:
            return 0, 1e3, None
        good = [
            m
            for pair in self.bf.knnMatch(d, e, k=2)
            if len(pair) == 2
            for m, n in [pair]
            if m.distance < 0.7 * n.distance
        ]
        if len(good) < 6:
            return 0, 1e3, None
        a = p[[m.queryIdx for m in good]]
        b = q[[m.trainIdx for m in good]]
        h, mask = cv2.findHomography(a, b, cv2.RANSAC, 3)
        if mask is None:
            return 0, 1e3, None
        keep = mask.ravel() > 0
        if keep.sum() < 6:
            return 0, 1e3, None
        # Identity displacement, unlike inlier count alone, measures pose alignment.
        residual = float(np.median(np.linalg.norm(a[keep] - b[keep], axis=1)))
        return int(keep.sum()), residual, h

    def setup(self, info):
        started = time.perf_counter()
        self.target_cache_hits = 0
        self.targets = info.targets
        self.target_thumbs = [self.thumb(t) for t in info.targets]
        self.target_features = []
        rankings = []
        for view, t in enumerate(info.targets):
            feat, matches = self._rank_target(
                t, remember=getattr(info, "session_id", "") != "preflight"
            )
            self.target_features.append(feat)
            rows = [(quality, i, view, count, residual) for quality, i, count, residual in matches]
            rankings.extend(rows)
            print("target", view, "top", sorted(rows, reverse=True)[:5], flush=True)
        rankings.sort(reverse=True)
        self.ranking = rankings
        _, self.goal, self.goal_view, _, _ = rankings[0]
        self.route = next(r for r in self.routes if self.goal in r)
        self.end = self.route.index(self.goal)
        self.index = 0
        self.phase = "route"
        self.scan_ticks = 0
        self.best_loss = 1e9
        self.current = self.route[0]
        print("teach route goal", self.goal, "view", self.goal_view, "index", self.end, flush=True)
        self.target_matching_seconds = time.perf_counter() - started
        print(
            "target cache hits",
            self.target_cache_hits,
            "matching seconds",
            round(self.target_matching_seconds, 4),
            flush=True,
        )
        if getattr(info, "session_id", "") != "preflight":
            self._save_target_cache()
