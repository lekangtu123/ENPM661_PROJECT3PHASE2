from peregrine.twins import SystemTwin
import os
import dupy_unreal as dupy
from peregrine.conf import global_settings
global_settings.Settings.BASE_OUTPUT_DIRECTORY = os.path.dirname(dupy.find_object(name=u'DuRTLFunctionLibrary', _class=dupy.find_object(name=u'Class')).get_cdo().call_function("GetCurrentScenarioFilename")) + "/Output"

class Turtlebot3Waffle(SystemTwin):
    def begin_play(self):
        super().begin_play()
        self.sensor_manager.start()

    def tick(self, delta_time):
        super().tick(delta_time)
        self.sensor_manager.capture_and_run_pipelines(self.sim_time)

    def end_play(self):
        super().end_play()
        self.sensor_manager.stop()