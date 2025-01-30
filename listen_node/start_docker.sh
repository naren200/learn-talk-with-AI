
# Create group with AUDIO_GID and add root to it
# if [ -n "${AUDIO_GID}" ]; then
#     groupadd -f -g ${AUDIO_GID} audio
#     usermod -a -G audio root
# fi
export AUDIO_GID=$(getent group audio | cut -d: -f3)


docker compose up -d 

docker exec -it "$(docker ps -q)" /bin/bash

