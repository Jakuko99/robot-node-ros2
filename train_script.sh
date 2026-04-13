#!/bin/bash
#SBATCH --mem=6G
#SBATCH --time=1:00:00
#SBATCH --output=TaskOutput.log
#SBATCH --tmp=10G

export SIM_FOLDER="/tmp/$SLURM_JOB_ID"
# export MODEL_PATH="$HOME/robotic_swarm/models"

mkdir -p /tmp/$SLURM_JOB_ID

module load apptainer/1.3.1-1
srun singularity run robot_swarm.sif
mv $SIM_FOLDER/ros_ws/export $WRKDIR/robotic_swarm
module purge
