#!/bin/bash

id=$(sudo docker ps | sed -n '2p' | cut -d " " -f 1)


echo $id

docker exec -it $id 'bash'
