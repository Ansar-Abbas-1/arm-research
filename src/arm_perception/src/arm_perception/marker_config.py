"""The table of markers the localizer looks for: id, measured size, dictionary (no ROS).

    dictionary: 4X4_1000            # default dictionary
    markers:
      - {id: 0,   size: 0.100}
      - {id: 984, size: 0.050, dictionary: 4X4_1000}

Sizes are the side of the black square including its border, in metres. Ids are unique across the table (they
name the TF frames). A listed id must exist in its dictionary (capacity check when a capacity function is given).
"""
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class MarkerSpec:
    id: int
    size: float
    dictionary: str


class MarkerConfig:
    def __init__(self, specs):
        self.specs = {}
        for s in specs:
            if s.id in self.specs:
                raise ValueError("marker id %d is listed twice (ids must be unique across the table)" % s.id)
            if not s.size > 0.0:
                raise ValueError("marker %d: size must be positive, got %r" % (s.id, s.size))
            self.specs[s.id] = s
        if not self.specs:
            raise ValueError("the marker table is empty")

    @classmethod
    def from_dict(cls, d, capacity=None):
        """capacity: optional function dictionary name -> number of ids, to reject ids the dictionary does not have."""
        default = d.get("dictionary")
        specs = []
        for i, m in enumerate(d.get("markers") or []):
            if "id" not in m or "size" not in m:
                raise ValueError("markers[%d] needs id and size: %r" % (i, m))
            name = m.get("dictionary", default)
            if not name:
                raise ValueError("marker %s has no dictionary and no default dictionary is set" % m["id"])
            spec = MarkerSpec(int(m["id"]), float(m["size"]), str(name))
            if capacity is not None:
                n = capacity(spec.dictionary)
                if not 0 <= spec.id < n:
                    raise ValueError("marker id %d does not exist in %s (it has %d ids, 0 to %d)" % (spec.id, spec.dictionary, n, n - 1))
            specs.append(spec)
        return cls(specs)

    @classmethod
    def load(cls, path, capacity=None):
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f) or {}, capacity)

    def dictionaries(self):
        """Distinct dictionaries in use, sorted (the detector runs once per dictionary)."""
        return sorted({s.dictionary for s in self.specs.values()})

    def spec(self, dictionary, marker_id):
        """The entry for a marker read in `dictionary`, or None if it is not in the table (or from another dictionary)."""
        s = self.specs.get(int(marker_id))
        return s if s is not None and s.dictionary == dictionary else None
