# fsm.py



# ++++++++++++++++++ Imports and Installs ++++++++++++++++++ #
import asyncio
from fsm.state_processes.state_deploy import StateDeploy
from fsm.state_processes.state_bootup import StateBootup
from fsm.state_processes.state_orient import StateOrient
from fsm.state_processes.state_detumble import StateDetumble


# ++++++++++++++++++++ Class Definition ++++++++++++++++++++ #
class FSM:
    def __init__(self, dp_obj, logger, config, deployment_switch,
                 tca, rx0, rx1, tx0, tx1):
        self.dp_obj = dp_obj    # object of type DataProcess
        self.logger = logger    # logging status of FSM states
        self.deployment_switch = deployment_switch
        self.state_objects = {
            "bootup"    : StateBootup(dp_obj, logger),
            "detumble"  : StateDetumble(dp_obj, logger, tca),
            "deploy"    : StateDeploy(dp_obj, logger, deployment_switch),
            "orient"    : StateOrient(dp_obj, logger, config, tca, rx0, rx1, tx0, tx1),
        }
        self.curr_state_name = "bootup"
        self.curr_state_object = self.state_objects["bootup"]
        self.curr_state_run_asyncio_task = asyncio.create_task(self.curr_state_object.run())
        self.deployed = False
    
    def set_state(self, new_state_name):
        """
        This function is called when we switch states from execute_fsm()
        """
        # Stop current state's background task
        # OK that orient may be in the future be stopped by detumble
        if self.curr_state_run_asyncio_task is not None:
            self.curr_state_object.stop()
            self.curr_state_run_asyncio_task.cancel()
            self.curr_state_run_asyncio_task = None

        self.curr_state_name = new_state_name
        self.curr_state_object = self.state_objects[new_state_name]
        self.curr_state_run_asyncio_task = asyncio.create_task(self.curr_state_object.run())

    def execute_fsm_step(self):
        """
        This function runs a single execution of the finite state machine (fsm)
        It checks its current state and data points and sees if we 
        need to change state, take action, etc.
        Note: because we pass in db_obj, its data variable will update 
        automatically if any changes are made for that db_obj
        """
        
        # NOTE: Emergency override for low battery and power consumption is handled in main.py
        # NOTE: Need to introduce emergency detumble post-first detumble, then continue where left off

        # Startup → Detumble
        if self.curr_state_name == "bootup" and self.curr_state_object.is_done():
            self.set_state("detumble")
            return 0
        
        # Emergency Detumble
        if self.dp_obj.data["data_imu_av_magnitude"] > 1:
            # Don't wait for other state to be done, shut it off immediately
            self.set_state("detumble")
            return 0

        # Detumble → Deploy
        if self.curr_state_name == "detumble" and self.curr_state_object.is_done():
            if self.deployed and self.dp_obj.data["data_batt_volt"] > 6:
                self.set_state("orient")
            elif not self.deployed and self.dp_obj.data["data_batt_volt"] > 7:
                self.deployed = True
                self.set_state("deploy")
            else:
                # Let the main file know we need to charge a bit more
                return -1  

        # Deploy → Orient
        if self.curr_state_name == "deploy" and self.curr_state_object.is_done() and self.dp_obj.data["data_batt_volt"] > 6:
            self.set_state("orient")
            return 0
        elif self.curr_state_name == "deploy" and self.curr_state_object.is_done() and self.dp_obj.data["data_batt_volt"] < 6:
            # Let the main file know we need to charge a bit more
            return -1