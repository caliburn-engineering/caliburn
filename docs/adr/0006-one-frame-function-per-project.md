# One frame function per project

Each project defines what a **frame** is in exactly one function: the order in which the setpoint, controller, actuators and physical system advance. The application and every test harness call that function; none writes out its own copy. A harness decides what to measure and when to disturb the system, and the frame function decides what a frame is. The same rule applies to any fact about the physical system: it is written down in one place.

Ball-balancer learned this the hard way ([#30](https://github.com/caliburn-engineering/caliburn/issues/30)). Five hand-written copies of its frame loop drifted apart. The harnesses read the servo rate before the step and the application read it after, a factor of 1.40 at 60 Hz, so a passing test suite was measuring a plate that was not the one that shipped. Letting each harness keep its own loop looked more flexible, and was rejected because such a harness gives evidence about itself, not about the application.
