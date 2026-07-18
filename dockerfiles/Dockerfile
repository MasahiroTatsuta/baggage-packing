FROM ubuntu:24.04

ENV PYTHONUNBUFFERED=TRUE \
    PYTHONDONTWRITEBYTECODE=TRUE \
    TZ="Asia/Tokyo" \
    LANG=ja_JP.UTF-8 \
    LANGUAGE=ja_JP:en \
    PATH=$PATH:/opt/conda/bin \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get -qq -y update && \
    apt-get -y install \
        wget \
        tzdata \
        locales \
        zip \
        unzip \
        libgl1-mesa-dev \
        build-essential \
        python3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && cp /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

# install miniconda
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-py312_25.7.0-2-Linux-x86_64.sh -P ./downloads/ && \
    bash ./downloads/Miniconda3-py312_25.7.0-2-Linux-x86_64.sh -b -p /opt/conda && \
    rm -rf ./downloads

# install additional libraries
RUN pip install gymnasium==1.2.3 pybullet==3.2.7 pillow==10.3.0
RUN pip install torch==2.7.0+cpu --extra-index-url https://download.pytorch.org/whl/cpu

RUN rm -rf ~/.cache/pip
