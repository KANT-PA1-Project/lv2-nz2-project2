"""Structural preflight. This is not a collision or dynamic-stability test."""
import math
import xml.etree.ElementTree as ET


def inspect_urdf(path):
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("Root element must be robot; expand xacro first")
    links = [x.get("name") for x in root.findall("link")]
    if not links or None in links or len(links) != len(set(links)):
        raise ValueError("Link names must be present and unique")
    parents, children, names, movable = {}, {}, set(), []
    for j in root.findall("joint"):
        name, kind = j.get("name"), j.get("type")
        if not name or name in names:
            raise ValueError("Joint names must be present and unique")
        names.add(name)
        p, c = j.find("parent"), j.find("child")
        if p is None or c is None:
            raise ValueError(f"{name}: missing parent/child")
        parent, child = p.get("link"), c.get("link")
        if parent not in links or child not in links or parent == child or child in parents:
            raise ValueError(f"{name}: invalid link relationship")
        parents[child] = parent
        children.setdefault(parent, []).append(child)
        if kind not in ("fixed", "revolute", "continuous", "prismatic"):
            raise ValueError(f"{name}: unsupported joint type {kind}")
        if kind != "fixed":
            lim = j.find("limit")
            if lim is None:
                raise ValueError(f"{name}: missing limit")
            required = ["effort", "velocity"] + ([] if kind == "continuous" else ["lower", "upper"])
            try:
                values = {k: float(lim.attrib[k]) for k in required}
            except (KeyError, ValueError) as exc:
                raise ValueError(f"{name}: invalid limit fields") from exc
            if not all(math.isfinite(v) for v in values.values()) or values["effort"] <= 0 or values["velocity"] <= 0:
                raise ValueError(f"{name}: invalid effort/velocity")
            if kind != "continuous" and values["lower"] >= values["upper"]:
                raise ValueError(f"{name}: invalid range")
            movable.append({"name": name, "type": kind, **values, "mimic": j.find("mimic") is not None})
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError("URDF must have exactly one root")
    seen, pending = set(), [next(iter(roots))]
    while pending:
        link = pending.pop()
        if link in seen:
            raise ValueError("Cycle found")
        seen.add(link)
        pending.extend(children.get(link, []))
    if seen != set(links):
        raise ValueError("Disconnected or cyclic links")
    warnings = [f"{x.get('name')}: inertial missing" for x in root.findall("link") if x.find("inertial") is None]
    return {"robot": root.get("name"), "root": next(iter(roots)), "links": links, "movable_joints": movable, "warnings": warnings}
