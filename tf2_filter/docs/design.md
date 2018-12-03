# Design Notes

Structurally, both a standalone implementation as well as a filter node will be provided. The latter is ready-to-use, and can even be composed in, the former can be embedded into existing nodes (such as the agent) to reduce latency.

The processing itself is trivial: Message in, check frame_id and child_frame_id, filtered message out.

The only moderately difficult part is knowing which parent-child pairs to pass through. In the general case, this requires looking at the robot model. And there might be parts that are only resolved at runtime.

In the first instance, we simply avoid this by making it a configuration parameter. For many simple systems (which we are targeting), setting this will be straightforward.
