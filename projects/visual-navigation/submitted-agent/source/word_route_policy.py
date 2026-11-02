"""Route-only experiment: broader retrieval with unchanged verification/controller."""
import pose_agent
from word_visual_route import shorten_route
from position_handoff_policy import MyAgent as Working

class MyAgent(Working):
    def setup(self, info):
        original=pose_agent.shorten_route
        try:
            pose_agent.shorten_route=shorten_route
            super().setup(info)
        finally:
            pose_agent.shorten_route=original

    def parent_mode(self, obs, index, reason, until=None):
        # An old visual anchor may end its trusted submap behind route progress.
        # Replaying that completed edge can return IDLE forever on one image.
        # Keep the existing visual servo, but aim at the current unpassed reference.
        if reason == "align before uncertain local motion edge" and index < self.index:
            print("FORWARD HANDOFF", obs.step, index, "->", self.index, flush=True)
            index = self.index
            until = min(self.end, max(index, until if until is not None else index))
        return super().parent_mode(obs, index, reason, until=until)
