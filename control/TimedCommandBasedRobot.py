from wpilib import TimedRobot

from wpilib.command.scheduler import Scheduler
from wpilib.livewindow import LiveWindow


class TimedCommandBasedRobot(TimedRobot):
    '''
    The base class for a Command-Based Robot. To use, instantiate commands and
    trigger them.
    '''

    def startCompetition(self):
        super().startCompetition()

    def robotPeriodic(self):
        '''
        Run the scheduler regularly. If an error occurs during a competition,
        prevent it from crashing the program.
        '''
        scheduler = Scheduler.getInstance()
        try:
            scheduler.run()
        except Exception as error:
            if not self.ds.isFMSAttached():
                raise

            '''Just to be safe, stop all running commands.'''
            scheduler.removeAll()

            self.handleCrash(error)


    def testPeriodic(self):
        '''
        Test mode will not run normal commands, but motors can be controlled
        and sensors viewed with the SmartDashboard.
        '''

        LiveWindow.run()


    def handleCrash(self, error):
        '''
        Called if an exception is raised in the Scheduler during a competition.
        Writes an error message to the driver station by default. If you want
        more complex behavior, override this method in your robot class.
        '''

        self.ds.reportError(str(error), printTrace=True)
