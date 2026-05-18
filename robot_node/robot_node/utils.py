from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import math


def pos_to_map_index(
    map: OccupancyGrid,
    pos: tuple[float, float],
    transform: TransformStamped = None,
) -> tuple[int, int]:
    """
    Return index of cells, that correspond to current odometry position
    """
    if transform is None:
        transform = TransformStamped()

    if pos[0] < map.info.origin.position.x or pos[1] < map.info.origin.position.y:
        raise ValueError("Position is out of map bounds")

    m_res: float = map.info.resolution
    return (
        math.floor(
            abs((pos[0] + transform.transform.translation.x - map.info.origin.position.x) / m_res)
        ),
        math.floor(
            abs((pos[1] + transform.transform.translation.y - map.info.origin.position.y) / m_res)
        ),
    )


def map_index_to_pos(
    map: OccupancyGrid,
    index: tuple[int, int],
) -> tuple[float, float]:
    """
    Return position of cell with given index
    """
    row, col = index
    if row < 0 or row >= map.info.height or col < 0 or col >= map.info.width:
        raise ValueError("Map index is out of map bounds")

    m_res: float = map.info.resolution
    return (
        map.info.origin.position.x + ((col + 0.5) * m_res),
        map.info.origin.position.y + ((row + 0.5) * m_res),
    )
