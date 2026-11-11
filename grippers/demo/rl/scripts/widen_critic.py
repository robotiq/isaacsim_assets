"""Make a checkpoint of the 10-slot shape task usable on the 18-slot one: new input columns (zero weights) for the extra pieces in the critic.

    python widen_critic.py <model_*.pt> <out.pt> [--old 10 --new 18]

The actor does not see the pieces and is unchanged. The critic's input is the policy observation (270) followed by the privileged one:
the positions of the slots (3 each), the tool tip (3) and 4 flags. The columns of the new slots are inserted after the old ones.
The optimizer state is dropped."""

import argparse

import torch

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("dst")
ap.add_argument("--old", type=int, default=10)
ap.add_argument("--new", type=int, default=18)
args = ap.parse_args()

ck = torch.load(args.src, map_location="cpu", weights_only=False)
c = ck["critic_state_dict"]
at, n = 270 + 3 * args.old, 3 * (args.new - args.old)
for name in ("obs_normalizer._mean", "obs_normalizer._var", "obs_normalizer._std"):
    v = c[name]
    fill = torch.zeros(1, n) if name.endswith("_mean") else torch.ones(1, n)
    c[name] = torch.cat([v[:, :at], fill, v[:, at:]], dim=1)
w = c["mlp.0.weight"]
c["mlp.0.weight"] = torch.cat([w[:, :at], torch.zeros(w.shape[0], n), w[:, at:]], dim=1)
ck["optimizer_state_dict"] = {"state": {}, "param_groups": ck["optimizer_state_dict"]["param_groups"]}
torch.save(ck, args.dst)
print("critic input", w.shape[1], "->", c["mlp.0.weight"].shape[1], "->", args.dst)
