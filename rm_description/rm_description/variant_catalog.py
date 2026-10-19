"""Canonical robot descriptions exposed by rm_description launch files."""
from dataclasses import dataclass
import re
from typing import Dict, Tuple

XacroMappings = Tuple[Tuple[str, str], ...]

@dataclass(frozen=True)
class DescriptionModel:
    """Files and fixed xacro inputs for one supported robot model."""

    model_file: str
    rviz_file: str
    xacro_mappings: XacroMappings = ()
    dual_arm: bool = False

# Each complete arm_type selects its model file and fixed xacro inputs.
DESCRIPTION_MODELS: Dict[str, DescriptionModel] = {
    "63": DescriptionModel("rml_63.urdf", "rm_63.rviz"),
    "63-6f": DescriptionModel(
        "rml_63.urdf.xacro", "rm_63.rviz", (("link6_type", "Link6_6f"),)
    ),
    "63-6fb": DescriptionModel(
        "rml_63.urdf.xacro", "rm_63.rviz", (("link6_type", "Link6_6fb"),)
    ),
    "63_iii": DescriptionModel(
        "rml_63.urdf.xacro",
        "rm_63.rviz",
        (("link6_type", "Link6"), ("base_type", "base_link_III")),
    ),
    "63_iii-6fb": DescriptionModel(
        "rml_63.urdf.xacro",
        "rm_63.rviz",
        (("link6_type", "Link6_6fb"), ("base_type", "base_link_III")),
    ),
    "65": DescriptionModel("rm_65.urdf", "rm_65.rviz"),
    "65-6f": DescriptionModel(
        "rm_65.urdf.xacro", "rm_65.rviz", (("link6_type", "Link6_6f"),)
    ),
    "65-6fb": DescriptionModel(
        "rm_65.urdf.xacro", "rm_65.rviz", (("link6_type", "Link6_6fb"),)
    ),
    "75": DescriptionModel("rm_75.urdf", "rm_75.rviz"),
    "75-6f": DescriptionModel(
        "rm_75.urdf.xacro", "rm_75.rviz", (("link7_type", "Link7_6f"),)
    ),
    "75-6fb": DescriptionModel(
        "rm_75.urdf.xacro", "rm_75.rviz", (("link7_type", "Link7_6fb"),)
    ),
    "eco62": DescriptionModel(
        "rm_eco62.urdf.xacro", "rm_eco62.rviz"
    ),
    "eco63": DescriptionModel("rm_eco63.urdf", "rm_eco63.rviz"),
    "eco63-6fb": DescriptionModel(
        "rm_eco63.urdf.xacro", "rm_eco63.rviz", (("link6_type", "Link6_6fb"),)
    ),
    "eco65": DescriptionModel("rm_eco65.urdf", "rm_eco65.rviz"),
    "eco65-6f": DescriptionModel(
        "rm_eco65.urdf.xacro", "rm_eco65.rviz", (("link6_type", "Link6_6f"),)
    ),
    "eco65-6fb": DescriptionModel(
        "rm_eco65.urdf.xacro", "rm_eco65.rviz", (("link6_type", "Link6_6fb"),)
    ),
    "gen72": DescriptionModel("rm_gen72.urdf", "rm_gen72.rviz"),
    "gen72_ii": DescriptionModel(
        "rm_gen72_II.urdf", "rm_gen72.rviz"
    ),
    "rx75-6fb": DescriptionModel(
        "rm_rx75-6fb.urdf.xacro", "rm_rx75.rviz", dual_arm=True
    ),
    "rx75-6fb-v": DescriptionModel(
        "rm_rx75-6fb_v.urdf.xacro", "rm_rx75.rviz", dual_arm=True
    ),
}

GLB_ARM_TYPES = ("65", "75", "eco62", "eco63", "eco65", "rx75")
MODEL_FORMATS = ("auto", "glb", "stl")
GLB_MODELS = {
    arm_type: DescriptionModel(
        "imported/" + arm_type.replace("-", "_")
        + ("_standard" if "-" not in arm_type else "") + ".urdf.xacro",
        spec.rviz_file,
        dual_arm=spec.dual_arm,
    )
    for arm_type, spec in DESCRIPTION_MODELS.items()
    if arm_type.split("-")[0] in GLB_ARM_TYPES
}


# The complete compatibility surface.  Tests keep this list synchronized with
# the catalog and with the actual wrapper files.
LEGACY_DISPLAY_LAUNCHES: Dict[str, str] = {
    "rm_63_display.launch.py": "63",
    "rm_63_6f_display.launch.py": "63-6f",
    "rm_63_6fb_display.launch.py": "63-6fb",
    "rm_63_III_display.launch.py": "63_iii",
    "rm_63_III_6fb_display.launch.py": "63_iii-6fb",
    "rm_65_display.launch.py": "65",
    "rm_65_6f_display.launch.py": "65-6f",
    "rm_65_6fb_display.launch.py": "65-6fb",
    "rm_75_display.launch.py": "75",
    "rm_75_6f_display.launch.py": "75-6f",
    "rm_75_6fb_display.launch.py": "75-6fb",
    "rm_eco62_display.launch.py": "eco62",
    "rm_eco63_display.launch.py": "eco63",
    "rm_eco63_6fb_display.launch.py": "eco63-6fb",
    "rm_eco65_display.launch.py": "eco65",
    "rm_eco65_6f_display.launch.py": "eco65-6f",
    "rm_eco65_6fb_display.launch.py": "eco65-6fb",
    "rm_gen72_display.launch.py": "gen72",
    "rm_gen72_II_display.launch.py": "gen72_ii",
    "rm_rx75_6fb_display.launch.py": "rx75-6fb",
    "rm_rx75_6fb_v_display.launch.py": "rx75-6fb-v",
}

_ARM_TYPE_ALIASES = {
    '63': '63',
    'rm63': '63',
    'rml63': '63',
    '63iii': '63_iii',
    'rm63iii': '63_iii',
    'rml63iii': '63_iii',
    '65': '65',
    'rm65': '65',
    '75': '75',
    'rm75': '75',
    'eco62': 'eco62',
    'rmeco62': 'eco62',
    'eco63': 'eco63',
    'rmeco63': 'eco63',
    'eco65': 'eco65',
    'rmeco65': 'eco65',
    'gen72': 'gen72',
    'rmgen72': 'gen72',
    'gen72ii': 'gen72_ii',
    'rmgen72ii': 'gen72_ii',
    'rx75': 'rx75',
    'rmrx75': 'rx75',
}

_ARM_TYPE_ALIASES.update({
    alias + arm_type[len(family):].replace("-", ""): arm_type
    for alias, family in _ARM_TYPE_ALIASES.items()
    for arm_type in DESCRIPTION_MODELS
    if arm_type == family or arm_type.startswith(family + "-")
})


def normalize_arm_type(arm_type: str, controller_version: int = 3) -> str:
    """Normalize a complete selector and reject unsupported end-link versions."""
    if controller_version not in (3, 4):
        raise ValueError("controller_version must be 3 or 4.")
    if not isinstance(arm_type, str) or not arm_type.strip():
        raise ValueError("arm_type must be a non-empty string.")
    token = re.sub(r"[\s_-]+", "", arm_type.strip().casefold())
    if controller_version == 4:
        # Fourth-generation 6F names reuse the existing integrated-force models.
        token = re.sub(r"6f(v?)$", r"6fb\1", token)
    normalized = _ARM_TYPE_ALIASES.get(token)
    if normalized not in DESCRIPTION_MODELS:
        raise ValueError(
            f"Unsupported arm_type='{arm_type}'. Valid values: "
            + ", ".join(DESCRIPTION_MODELS)
        )
    return normalized


def resolve_model(arm_type: str, model: str = "auto") -> DescriptionModel:
    """Resolve one selector or raise an error suitable for launch output."""
    arm_type = normalize_arm_type(arm_type)
    model = model.strip().casefold()
    if model not in MODEL_FORMATS:
        raise ValueError(
            f"Unsupported model='{model}'. "
            f"Valid values: {', '.join(MODEL_FORMATS)}."
        )
    use_glb = model == "glb" or (model == "auto" and arm_type in GLB_MODELS)
    catalog = GLB_MODELS if use_glb else DESCRIPTION_MODELS
    try:
        return catalog[arm_type]
    except KeyError as exc:
        raise ValueError(
            f"No {model.upper()} model for arm_type='{arm_type}'. "
            f"Valid arm_type values: {', '.join(catalog)}."
        ) from exc


def resolve_gazebo_model(arm_type: str, model: str = "auto") -> str:
    """Use available GLB visuals with the existing simulation skeleton."""
    arm_type = normalize_arm_type(arm_type)
    resolve_model(arm_type, model)
    model = model.strip().casefold()
    return "glb" if model != "stl" and arm_type in GLB_MODELS else "stl"


def apply_visual_model(robot_description: str, arm_type: str, model: str = "auto") -> str:
    """Replace visuals only; preserve the caller's joints, collisions and physics."""
    if resolve_gazebo_model(arm_type, model) == "stl":
        return robot_description

    from copy import deepcopy
    from pathlib import Path
    import xml.etree.ElementTree as ET

    from ament_index_python.packages import get_package_share_directory
    import xacro

    spec = resolve_model(arm_type, "glb")
    source = Path(get_package_share_directory("rm_description")) / "urdf" / spec.model_file
    visuals = ET.fromstring(xacro.process_file(str(source)).toxml())
    robot = ET.fromstring(robot_description)
    links = {link.get("name"): link for link in robot.findall("link")}
    replaced = set()

    for source_link in visuals.findall("link"):
        if not source_link.findall("visual"):
            continue
        name = source_link.get("name")
        if name not in links:
            raise ValueError(f"GLB visual link '{name}' is missing from arm_type='{arm_type}'.")
        target = links[name]
        for visual in target.findall("visual"):
            target.remove(visual)
        for visual in source_link.findall("visual"):
            visual = deepcopy(visual)
            for mesh in visual.findall("geometry/mesh"):
                filename = mesh.get("filename", "")
                if filename.startswith("package://"):
                    package, relative = filename[len("package://"):].split("/", 1)
                    mesh.set("filename", (Path(get_package_share_directory(package)) / relative).as_uri())
            target.append(visual)
        replaced.add(name)

    # Gazebo's solid-color override hides the materials stored inside GLB.
    for gazebo in robot.findall("gazebo"):
        if gazebo.get("reference") in replaced:
            for material in gazebo.findall("material"):
                gazebo.remove(material)
    return ET.tostring(robot, encoding="unicode")
