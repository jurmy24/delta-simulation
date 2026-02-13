#!/usr/bin/env python3
"""
MuJoCo simulation runner for the delta robot.
Loads the scene, logs any errors, and launches the interactive viewer.
"""

import sys
import logging
from pathlib import Path

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('sim_errors.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


def main():
    # Get the directory where this script is located
    script_dir = Path(__file__).parent
    scene_path = script_dir / "scene.xml"
    
    logger.info(f"Loading MuJoCo model from: {scene_path}")
    
    try:
        import mujoco
    except ImportError as e:
        logger.error(f"Failed to import mujoco: {e}")
        logger.error("Install with: pip install mujoco")
        sys.exit(1)
    
    # Try to load the model
    try:
        logger.info("Parsing MJCF file...")
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        logger.info(f"Model loaded successfully!")
        logger.info(f"  - Bodies: {model.nbody}")
        logger.info(f"  - Joints: {model.njnt}")
        logger.info(f"  - Geoms: {model.ngeom}")
        logger.info(f"  - Actuators: {model.nu}")
        logger.info(f"  - DOF: {model.nv}")
        
    except mujoco.FatalError as e:
        logger.error(f"MuJoCo fatal error while loading model: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error loading model: {type(e).__name__}: {e}")
        sys.exit(1)
    
    # Create simulation data
    try:
        data = mujoco.MjData(model)
        logger.info("Simulation data initialized")
    except Exception as e:
        logger.error(f"Error creating simulation data: {e}")
        sys.exit(1)
    
    # Try a single simulation step to check for runtime errors
    try:
        logger.info("Running test simulation step...")
        mujoco.mj_step(model, data)
        logger.info("Test step completed successfully")
    except Exception as e:
        logger.error(f"Error during simulation step: {e}")
        sys.exit(1)
    
    # Launch the viewer
    try:
        import mujoco.viewer
        logger.info("Launching interactive viewer...")
        logger.info("Controls: Space=pause, Backspace=reset, Tab=UI, Ctrl+L=reload")
        mujoco.viewer.launch(model, data)
    except ImportError:
        logger.warning("mujoco.viewer not available, running headless simulation")
        logger.info("Running 1000 simulation steps...")
        for i in range(1000):
            mujoco.mj_step(model, data)
            if i % 100 == 0:
                logger.info(f"  Step {i}: time={data.time:.3f}s")
        logger.info("Simulation completed")
    except Exception as e:
        logger.error(f"Error launching viewer: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
