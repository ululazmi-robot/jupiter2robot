"""Validate the driver manifest without needing ROS.

Usage:  manifest_check [path/to/drivers.yaml]

Exits 0 when the manifest loads and validates, 1 with a list of problems
otherwise. Deliberately importable without rclpy so it runs anywhere.
"""

import sys


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    path = argv[0] if argv else None
    try:
        import jupiter_monitor_hub.manifest as mf
    except ImportError:
        print("jupiter_monitor_hub is not importable; run from an installed "
              "workspace or add src/ to PYTHONPATH", file=sys.stderr)
        return 2

    try:
        meta, drivers = mf.load(path)
    except FileNotFoundError as exc:
        print("manifest not found: %s" % exc, file=sys.stderr)
        return 1
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("manifest v%s  %s" % (meta.get("version"), meta["path"]))
    print("%d drivers" % len(drivers))
    for cat in mf.CATEGORIES:
        group = [d for d in drivers if d["category"] == cat]
        if not group:
            continue
        print("\n  %s" % cat)
        for d in group:
            print("    %-20s %-9s %-24s %s"
                  % (d["key"], d["backend"], d.get("probe", "none"),
                     d.get("primary_topic") or "-"))

    # Duplicate primary_topic is worth flagging, but it is NOT always a bug.
    # joint_state_broadcaster (and this repo's head_node) merge every joint onto
    # ONE /joint_states, so arm and head legitimately share it -- that is the ROS
    # convention, not a conflict. Rate measurement still wants a single owner, so
    # only the FIRST claimant should measure; the rest leave primary_topic empty.
    #
    # A genuine error is two drivers claiming a topic that is not a merged joint
    # stream, e.g. both claiming /cmd_vel.
    SHAREABLE = ("/joint_states",)

    seen = {}
    problems = []
    for d in drivers:
        t = d.get("primary_topic") or ""
        if not t:
            continue
        if t in seen:
            if t in SHAREABLE:
                problems.append(
                    "%s also claims %s (fine to share, but move it to "
                    "primary_topic: \"\" so the other keeps the rate reading)"
                    % (d["key"], t))
            else:
                problems.append("primary_topic %s claimed by both %s and %s"
                                % (t, seen[t], d["key"]))
        seen[t] = d["key"]

    if problems:
        print("\nwarnings:", file=sys.stderr)
        for p in problems:
            print("  " + p, file=sys.stderr)
        return 1
    print("\nok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
