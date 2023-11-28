FROM python:3.10

RUN mkdir /root/.mujoco
COPY mujoco210-linux-x86_64.tar.gz /root/.mujoco/
RUN cd /root/.mujoco && tar xzf /root/.mujoco/mujoco210-linux-x86_64.tar.gz
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin"

RUN pip install "cython<3"
RUN pip install numpy cffi lockfile
RUN pip install mujoco-py==2.1.2.14
RUN pip install wandb torch gym==0.21.0

RUN echo deb http://deb.debian.org/debian/ sid main contrib non-free non-free-firmware >> /etc/apt/sources.list
RUN apt-get -y update && apt-get install --no-install-recommends -y libglew-dev libosmesa6-dev xvfb patchelf ffmpeg cmake libgl1-mesa-dev libglu1-mesa-dev && apt-get clean

WORKDIR /app
COPY . .
RUN python -c "import mujoco_py" 
CMD ["python", "-u", "src/main.py"]
