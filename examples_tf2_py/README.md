# Python examples for tf2

This package has examples for using the `tf2_ros` API from python.
It shows how to broadcast and receive transforms between frames.
The example transforms might be seen on a small two wheel robot.

## Broadcasting Transforms

`tf2` publishes two types of transforms: static and dynamic.
Static transforms are constant, and never change once published.
Dynamic transforms are frequently changing, and re-published every time the do.

Both broadcasters can be run with

```
ros2 launch examples_tf2_py broadcasters.launch.xml
```

### `dynamic_broadcaster.py`

```
ros2 run examples_tf2_py dynamic_broadcaster
```

This is an example of a dynamic transform publisher.
Transforms showing the rotation of two wheels are published frequently.

### `static_broadcaster.py`

```
ros2 run examples_tf2_py static_broadcaster
```

This is an example of a static transform publisher.
The transforms to sensors on the robot are published once at startup, and then
never changed.

## Receiving Transforms

Transforms are broadcast to all listeners, who then use the data.
If a transform is not immediately available then users of the data may choose
to wait for it.

### `waits_for_transform.py`

```
ros2 run examples_tf2_py waits_for_transform
```

This example blocks until a transform is received.
It must be run with a `MultiThreadedExecutor` so the `TransformListener` can
execute callbacks for it's subscriptions.

### `async_waits_for_transforms.py`

```
ros2 run examples_tf2_py async_waits_for_transform
```

This example uses coroutines to wait for a transform.
The coroutine is suspended without blocking the executor, so it works with
a `SingleThreadedExecutor`.

### Frame Dumper

```
ros2 run examples_tf2_py frame_dumper
```

This example periodically outputs information about all frames it is aware of.
