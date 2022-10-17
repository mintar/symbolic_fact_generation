from dataclasses import dataclass
from typing import List

@dataclass
class Fact:
    name: str
    values: List[str]