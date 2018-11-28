# Introduction

## Overall Goal

The goal of this package is to reduce bandwidth between machines and CPU
load on transform users.

## Terms

 * *Frame ID:* The unique identifier of a coordinate frame.
 * *Transform:* Translation and rotation relating two coordinate frames.
 * *Transform chain:* A series of transforms without breaks, to relate
 coordinate frames.
 * *Transform user:* The node that makes use of transforms to realize
 functionality.
 * *Filter:* A pair of frame_id's, the specifies a transform chain of interest for the user.

## System Use Cases

1) A semi-autonomous drone swarm, where drones receive information about other drones
2) An autonomous domestic robot that does internal sensor-frame to base_link transformations, and also sends its map-pose, mainly for debugging

## Assumptions

 * The user of the filtered transform is not on the same machine as the generator. Otherwise there are better approaches to exchange the data.
 * TF users do not actually need all data. Otherwise there's nothing to filter.
 * Once a transform chain exists, its constituent links stay the same (simplicity)
 * Latency is important (otherwise we would use the `buffer_server`)
 * Bandwidth and/or CPU is scarce (otherwise we would send everything)

# Requirements


## Data

  * As a user, I want the output to be TFMessage, so that I can simply remap /tf->/tf_filtered for the target node and everything else stays the same.
  * As a CPU-limited user, I would like to receive a transform chain collapsed to a single transform so that I don't have to compute it myself.

## Configuration

  * As an integrator I want to specify to be able to specify the target chain in the configuration, so that the target nodes don't have to be modified.

  * As a developer, I also want to be able to specify the (or a) chain at runtime, so that I can specify it based on runtime information.

  * As a performance engineer, I want one filter node to be able to process multiple transform chains so that the resource use associated with receiving and processing incoming data occurs only once.
