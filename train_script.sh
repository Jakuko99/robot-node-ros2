#!/bin/bash
#SBATCH --mem=4G
#SBATCH --time=0:50:00
#SBATCH --output=TaskOutput.log
#SBATCH --tmp=10G

export SIM_FOLDER="/tmp/$JOB_ID"
export PROJ_FOLDER="$HOME/robotic_swarm"
# export MODEL_PATH="$PROJ_FOLDER/models"
export MESH_PATH="$PROJ_FOLDER/meshes"

mkdir -p $SIM_FOLDER
# module load apptainer/1.3.1-1
echo Starting job $JOB_ID ...

singularity run robot_swarm.sif
mv $SIM_FOLDER/ros_ws/export $PROJ_FOLDER/export-$JOB_ID

# module purge
rm -r $SIM_FOLDER
echo Job complete.