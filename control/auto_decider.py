from commands.auto.switch_and_scale import SwitchAndScale
from commands.auto.switch_only import SwitchOnlyCenter, SwitchOnlySide
from commands.auto.vault import VaultOnly
from commands.pursuit_drive import PursuitDriveCommand
from control.game_data import Side


class Desire:
    VAULT = 0
    SWITCH = 1
    SCALE = 2
    CROSS = 3


class AutoDecider:
    def __init__(self, desires=set()):
        self.desires = desires

    def can_do(self, robot_side, switch_side, scale_side):
        pass

    def decide(self, robot_side, switch_side, scale_side):
        if robot_side == Side.CENTER and Desire.SCALE in self.desires:
            print("Invalid desire SCALE for position CENTER")
            self.desires.remove(Desire.SCALE)

        if Desire.VAULT in self.desires and robot_side != Side.CENTER:
            print("Invalid desire VAULT for non-center position")
            self.desires.remove(Desire.VAULT)

        if Desire.SWITCH in self.desires:
            if robot_side == Side.CENTER:
                return SwitchOnlyCenter
            if robot_side == switch_side:
                return SwitchOnlySide

            if Desire.SCALE in self.desires:
                return SwitchAndScale

            if Desire.VAULT in self.desires:
                return None  # VaultAndSwitch

        if Desire.VAULT in self.desires:
            return VaultOnly

        if Desire.CROSS in self.desires:
            return PursuitDriveCommand

        return None


