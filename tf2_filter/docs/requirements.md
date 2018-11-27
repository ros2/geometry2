# Introduction

## Overall Goal

The goal of this package is to reduce bandwidth between machines and CPU
load on TF-data receivers.

## Use Cases

1) A semi-autonomous drone swarm, where drones receive information about
other drones
2) An autonomous domestic robot that does internal sensor-frame to base_link
transformations, and also sends its map-pose, mainly for debugging

## Assumptions

 * Receivers do not actually need all TF data. Otherwise there's nothing to filter.
 * Once a transform exists, its constituent links stay the same (simplicity)
 * Latency is important (otherwise we would use the buffer_server)
 * Bandwidth is scarce (otherwise we would send everything)

# Requirements List

## Product filter output as TFMessage on a configurable topic

TFMessage is the standard data-type, and we might need multiple
filters, hence a configurable topic.

This make it so that we can simply remap /tf for the target node,
and it will receive the filtered input instead of the standard
topic.

## Support multiple filters per node

Maybe not initially, but would make sense efficiency-wise, to reduce
resource use for inbound data processing.

## Specify a specific chain to filter at configuration time

Simplest case: Target chain is specified in a configuration file.

## Specify chain to filter using a service

At runtime, make a service request to specify which chain to filter.
