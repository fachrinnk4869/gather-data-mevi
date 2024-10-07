# Camera Data Collection Project

This project is designed to gather data from a connected camera using Docker and Docker Compose. Follow the instructions below to set up and run the project.

## Prerequisites

Ensure the following are installed on your system:

- **Docker**: [Install Docker](https://docs.docker.com/get-docker/).
- **Docker Compose**: [Install Docker Compose](https://docs.docker.com/compose/install/).
- A camera connected to your machine (e.g., Brio, RealSense, ZED).

## Setup

### 1. Change directory of the Repository

First, change directory in jetson:

```bash
cd ~/catkin_ws
```

### 2. Running the Project

#### Step 1: Run the `run.bash` Script

The `run.bash` script uses Docker Compose to launch the necessary services. Simply execute the script as follows:

```bash
bash run.bash
```

This will start the Docker container and any other services configured in your `docker-compose.yml` file.

Once the container is running, you can execute the Python script inside the Docker container to start gathering data from your camera

#### Step 2: Run the Python Script Inside Docker

Once inside the Docker container, run the following Python script to gather data from your camera:

```bash
python3 gather_data_camera.py
```

### 3. Stopping the Containers

To stop and remove the running containers when you're done, run:

```bash
sudo docker compose down
```

This will stop and remove the containers, networks, and volumes defined in the `docker-compose.yml` file.

## Troubleshooting

- If you encounter issues accessing the camera from within the container, check that the camera drivers are correctly installed and accessible in the Docker container.

```

This version assumes that your `run.bash` script uses `docker compose` commands to manage the container lifecycle. Let me know if you need further adjustments!
