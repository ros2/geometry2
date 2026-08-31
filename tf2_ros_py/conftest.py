# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from rmw_test_fixture_implementation import rmw_test_isolation_start
from rmw_test_fixture_implementation import rmw_test_isolation_stop


@pytest.fixture(autouse=True, scope='session')
def rmw_isolation():
    """Start RMW isolation before any ROS context is created."""
    rmw_test_isolation_start()
    yield
    rmw_test_isolation_stop()
