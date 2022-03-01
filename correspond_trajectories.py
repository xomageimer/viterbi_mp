from argparse import ArgumentParser
import json


if __name__ == "__main__":
    parser = ArgumentParser(description="Compare two trajectories")
    parser.add_argument("true", help="true trajectory file")
    parser.add_argument("ans", help="your trajectory file")
    args = parser.parse_args()

    with open(args.true) as f:
        true_traj = json.load(f)

    with open(args.ans) as f:
        ans_traj = json.load(f)

    if len(true_traj) != len(ans_traj):
        print("Expected trajectory of length {},".format(len(true_traj)) +
              "but got trajectory of length {}".format(len(ans_traj)))
        exit(1)

    n = len(true_traj)
    for i, p, q in zip(range(n), true_traj, ans_traj):
        if p != q:
            print("Wrong position on iteration {}".format(i))
            exit(1)

    print("OK")
