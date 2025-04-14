from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message
from alphabot_agent.alphabotlib.AlphaBot2 import AlphaBot2
import asyncio
import os
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("AlphaBotAgent")

# Enable SPADE and XMPP specific logging
for log_name in ["spade", "aioxmpp", "xmpp"]:
    log = logging.getLogger(log_name)
    log.setLevel(logging.INFO)
    log.propagate = True


class AlphaBotAgent(Agent):
    class XMPPCommandListener(CyclicBehaviour):
        async def on_start(self):
            self.ab = AlphaBot2()

        async def run(self):
            msg = await self.receive(timeout=100)
            if msg:
                logger.info(f"[Behavior] Received command ({msg.sender}): {msg.body}")
                await self.process_command(msg.body)

                # Send a confirmation response
                reply = Message(to=str(msg.sender))
                reply.set_metadata("performative", "inform")
                reply.body = f"Executed command: {msg.body}"
                await self.send(reply)

        async def process_command(self, command):
            command = command.strip().lower()
            command = command.strip().lower()
            args = command.split()[1:]
            command = command.split()[0]

            if command == "forward":
                distance = args[0]
                distance = float(distance)
                logger.info(f"[Behavior] Moving forward for {distance} mm")
                self.ab.safeForward(mm=distance)

            elif command == "turn":
                angle = args[0]
                angle = float(angle)
                logger.info("[Behavior] Turning...")
                self.ab.turn(angle=angle)
            elif command == "full_calibration":
                logger.info("Starting full calibration")
                self.ab.fullCalibration()
            elif command == "calibrate_turn":
                logger.info("Calibrating turn...")
                self.ab.calibrateTurn()
            elif command == "calibrate_sensors":
                logger.info("[Behavior] Calibrating sensors...")
                self.ab.calibrateTRSensors()
            elif command == "calibrate_forward":
                logger.info("[Behavior] Calibrating forward...")
                self.ab.calibrateForward()
            elif command == "calibrate_forward_correction":
                logger.info("Calibrating forward correction")
                self.ab.calibrateForwardCorrection()

            elif command.startswith("motor "):
                try:
                    _, left, right = command.split()
                    left_speed = int(left)
                    right_speed = int(right)
                    logger.info(
                        f"[Behavior] Setting motor speeds to {left_speed} (left) and {right_speed} (right)..."
                    )
                    self.ab.setMotor(left_speed, right_speed)
                    await asyncio.sleep(2)
                    self.ab.stop()
                except (ValueError, IndexError):
                    logger.error(
                        "[Behavior] Invalid motor command format. Use 'motor <left_speed> <right_speed>'"
                    )

            elif command == "stop":
                logger.info("[Behavior] Stopping...")
                self.ab.stop()

            else:
                logger.warning(f"[Behavior] Unknown command: {command}")

    async def setup(self):
        # Add command listener behavior
        command_behavior = self.XMPPCommandListener()
        self.add_behaviour(command_behavior)


import asyncio


async def main():
    xmpp_domain = os.environ.get("XMPP_DOMAIN", "prosody")
    xmpp_username = os.environ.get("XMPP_USERNAME", "alpha-pi-zero-agent")
    xmpp_jid = f"{xmpp_username}@{xmpp_domain}"
    xmpp_password = os.environ.get("XMPP_PASSWORD", "top_secret")
    try:
        agent = AlphaBotAgent(
            jid=xmpp_jid, password=xmpp_password, verify_security=False
        )

        await agent.start(auto_register=True)
        
        try:
            while agent.is_alive():
                await asyncio.sleep(100)  # Log every 10 seconds that agent is alive
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")
            await agent.stop()
    except Exception as e:
        logger.error(f"Error starting agent: {str(e)}", exc_info=True)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logger.critical(f"Critical error in main loop: {str(e)}", exc_info=True)
