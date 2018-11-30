# Introduction

Simple package for filtering a TF stream by link.

Subscribes to `/tf` and publishes to `/tf_filtered`, by default.

## Synopsis

`ros2 run tf2_filter tf2_filter_node base_link arm1`

This will pass through all transfroms from `base_link` to `arm1`.

For chains with multiple links, simple give all links in order as arguments.

## Example

You can run `tf2_filter_example_publisher.py` to give you an example /tf stream (with empty transforms...). It publishes two transforms. With the TF2Filter invocation above, only one will remain.

# License

tf2_filter is licensed under the [Apache Public License v2](LICENSE).
