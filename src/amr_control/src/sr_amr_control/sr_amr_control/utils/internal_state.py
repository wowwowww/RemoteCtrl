from dataclasses import dataclass


@dataclass
class InternalState:
  movement_no: int = 0

  def generate_movement_no(self) -> int:
    self.movement_no += 1
    return self.movement_no
