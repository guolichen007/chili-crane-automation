"""Planning payload is observed indexed evidence; omitted indices remain UNKNOWN."""
from .pit_surface import SurfaceCell


ENCODING = 'OBSERVED_INDEXED_V2'


def sparse_cells(snapshot):
    return tuple(c for c in snapshot.cells if c.observed)


def expand_sparse(cells, width, height, maximum):
    count = width*height
    if not 0 < count <= maximum or len(cells) > count:
        raise ValueError('SPARSE_GRID_CAPACITY_INVALID')
    dense = [SurfaceCell(i) for i in range(count)]
    seen = set()
    for cell in cells:
        if type(cell.index) is not int or not 0 <= cell.index < count or cell.index in seen or not cell.observed:
            raise ValueError('SPARSE_INDEX_OR_OBSERVATION_INVALID')
        seen.add(cell.index)
        dense[cell.index] = cell
    return tuple(dense)
