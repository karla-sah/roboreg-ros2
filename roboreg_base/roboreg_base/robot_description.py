import re

import pytorch_kinematics as pk
from roboreg.core.robot import RobotData

_FILE_URI = re.compile(r"""filename=(["'])file://([^"']*)\1""")
_SHARE_SPLIT = re.compile(r"/share/([^/]+)/(.+)$")


def to_package_uris(urdf: str) -> str:
    def _rewrite(match: re.Match) -> str:
        quote, path = match.group(1), match.group(2)
        share = _SHARE_SPLIT.search(path)
        if share is None:
            raise ValueError(
                f"Mesh path 'file://{path}' is not inside a ROS package share "
                "directory and cannot be converted to a package:// URI."
            )
        package, relative_path = share.group(1), share.group(2)
        return f"filename={quote}package://{package}/{relative_path}{quote}"

    return _FILE_URI.sub(_rewrite, urdf)


def chain_joint_names(robot_data: RobotData) -> list[str]:
    chain = pk.build_serial_chain_from_urdf(
        robot_data.urdf,
        end_link_name=robot_data.end_link_name,
        root_link_name=robot_data.root_link_name,
    )
    return chain.get_joint_parameter_names()
