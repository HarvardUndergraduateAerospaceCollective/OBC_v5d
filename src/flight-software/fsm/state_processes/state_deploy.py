# state_deploy.py


# ++++++++++++++ Imports/Installs ++++++++++++++ #
import asyncio


# ++++++++++++++ Functions: Helper ++++++++++++++ #
class StateDeploy:
    def __init__(self, dp_obj, logger, config, deployment_switch):
        """
        Initialize the class object
        """
        self.dp_obj = dp_obj
        self.logger = logger
        self.config = config
        self.deployment_switch = deployment_switch
        self.finished_burn = False
        self.running = False
        self.done = False

    async def run(self):
        """
        Run the deployment sequence asynchronously.

        Performs multiple burn attempts with configurable delay between attempts.
        Uses config.deploy_max_attempts for number of burn attempts.
        Uses config.deploy_retry_delay for delay between attempts (seconds).
        Uses config.deploy_burn_duration for each burn duration.
        """
        # Reset state flags on re-entry (e.g., after emergency detumble during deploy)
        self.finished_burn = False
        self.done = False
        self.running = True
        while self.running:
            await asyncio.sleep(1)
            # Burn the wire if not already done to release the antennas
            if not self.finished_burn:
                if self.deployment_switch:
                    num_burns = getattr(self.config, "deploy_max_attempts", 3)
                    retry_delay = getattr(self.config, "deploy_retry_delay", 60.0)

                    for attempt in range(num_burns):
                        self.logger.info(
                            f"[Deploy] Burn attempt {attempt + 1}/{num_burns}, duration={self.config.deploy_burn_duration}s"
                        )
                        self.deployment_switch.burn(self.config.deploy_burn_duration)

                        # Don't wait after the last attempt
                        if attempt < num_burns - 1:
                            self.logger.info(
                                f"[Deploy] Waiting {retry_delay}s before next attempt"
                            )
                            await asyncio.sleep(retry_delay)

                self.finished_burn = True
                self.logger.info("[Deploy] All burn attempts completed")
            self.done = True
            self.running = False

    def stop(self):
        """
        Used by FSM to manually stop run()
        """
        self.running = False

    def is_done(self):
        """
        Checked by FSM to see if the run() completed on its own
        If it did complete, it shuts down the async task run()
        """
        return self.done
