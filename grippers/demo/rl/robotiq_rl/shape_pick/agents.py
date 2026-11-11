"""PPO settings of the shape pick: those of the tactile pick, with their own experiment name."""

from isaaclab.utils import configclass

from ..tactile_pick.agents import TactilePickPPORunnerCfg


@configclass
class ShapePickPPORunnerCfg(TactilePickPPORunnerCfg):
    experiment_name = "tactile_shape_pick"
