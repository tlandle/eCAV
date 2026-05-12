#FROM ubuntu:22.04
FROM nvidia/cuda:12.1.0-base-ubuntu22.04

WORKDIR /root

RUN apt-get update && apt-get upgrade -y && apt-get install -y software-properties-common

RUN	add-apt-repository ppa:deadsnakes/ppa && \
	apt-get install -y python3.10 && \
	apt-get install -y python3.10-tk && \
	apt-get install -y ffmpeg libsm6 libxext6 && \
	apt-get install -y python3-pip && \
	apt-get install -y python3-apt

RUN python3.10 -m pip install --upgrade pip && python3.10 -m pip install "setuptools<81"

COPY requirements_3_10.txt .

RUN python3.10 -m pip install --ignore-installed blinker && \
    python3.10 -m pip install torch==2.9.1 torchvision==0.24.1 && \
    python3.10 -m pip install -r requirements_3_10.txt -f https://data.pyg.org/whl/torch-2.9.1+cu128.html

RUN apt-get install -y libglfw3-dev

RUN apt-get install -y libxcb-*

RUN export DISPLAY=:0.0

COPY . .

# Install opencood from worldfusion submodule
RUN python3.10 -m pip install -e ecav/worldfusion

# Build Cython extension (box_overlaps)
RUN cd ecav/worldfusion && python3.10 opencood/utils/setup.py build_ext --inplace

# NOTE: pcdet_utils CUDA extensions (iou3d_nms, roiaware_pool3d, pointnet2) are NOT built here.
# Container actors run in distributed mode with carla_world=None, so they use the base
# PerceptionManager which does not require these extensions. CUDA extensions are only needed
# for non-distributed WorldFusion perception paths on the host.

EXPOSE 5555/tcp

# gRPC
EXPOSE 50051/tcp
EXPOSE 50052/tcp
EXPOSE 50053/tcp
EXPOSE 50054/tcp
# probably just want more ports because the edge will need them
EXPOSE 50101-50512/tcp

# Carla
EXPOSE 2000/tcp

RUN python3.10 -m grpc_tools.protoc -I./ecav/protos --python_out=. --grpc_python_out=. ./ecav//protos/ecloud.proto
