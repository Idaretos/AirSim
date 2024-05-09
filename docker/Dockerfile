ARG BASE_IMAGE=adamrehn/ue4-runtime
FROM $BASE_IMAGE

USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y --no-install-recommends \
	rsync \
    sudo \
    wget \
    x11-xserver-utils \
    git

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    adduser ue4 sudo 

WORKDIR /home/ue4
RUN git config --global user.email "arica4@snu.ac.kr"
RUN git config --global user.name "Jihwan-Kimm"
RUN git config --global http.sslVerify false

RUN git clone --progress https://<================>/Jihwan-Kimm/UnrealEngine.git
WORKDIR /home/ue4/UnrealEngine
RUN ./Setup.sh
RUN ./GenerateProjectFiles.sh
RUN make

WORKDIR /home/ue4
RUN git clone --progress https://github.com/<================>/AirSim.git
WORKDIR /home/ue4/AirSim 
RUN	./setup.sh
RUN	./build.sh

COPY start.sh /start.sh
RUN chmod a+rwx /start.sh
CMD ["sh","/start.sh"]