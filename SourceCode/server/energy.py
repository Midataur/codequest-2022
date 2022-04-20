from codequest22.stats import energy

class EnergyTile:

    def __init__(self, position, amount) -> None:
        self.position = position
        self.amount = amount
        self.max_delay = energy.DELAY
        self.per_tick = energy.PER_TICK
        self.cur_delay = 0
    
    def __repr__(self) -> str:
        return f"{{\"{self.__class__.__name__}\": {[self.position, self.amount]}}}"
    
    def to_json(self):
        return {
            "classname": "EnergyTile",
            "position": self.position,
            "amount": self.amount,
        }
    
    @classmethod
    def from_json(cls, data):
        return EnergyTile(tuple(data["position"]), data["amount"])
