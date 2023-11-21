# AOS Overview

## Concepts

- **Factory** - holds all event loops that need to get executed together
- **Logger** - based on `Watcher`/ `Timer`/ `Fetcher` interface
- **EventLoop** - always associated with a single node (computer)
	- Name of `EventLoop` is same as name of application

## Core `EventLoop` `Typed` Interface

```cpp
// Send messages.
Sender<T> EventLoop::MakeSender<T>(std::string_view name);
// Watch for a message name and trigger a callback upon seeing one.
void EventLoop::MakeWatcher<T>(std::string_view name,
							   std::function<void(const T&)> callback);
// Grab the latest message from the queue.
Fetcher<T> EventLoop::MakeFetcher<T>(std::string_view name);
// Trigger a callback after specified time has elapsed.
TimerHandler *EventLoop::AddTimer(std::function<void()> callback);
// Create timer with initial time and repeat interval
// to trigger callback each time.
void TimerHandler::Setup(monotonic_clock::time_point base,
						 chrono::nanoseconds repeat)
// Read the current clock.
monotonic_clock::time_point EventLoop::monotonic_now();
```

## Core `EventLoop` `Reflection` Interface

```cpp
// List of channels - pull one out or find one
const Channel *channel = loop.configuration()->channels()->Get(i);
// Sender - pass channel in, and this will let
// you send chunks of memory (well-formed flatbuffers).
RawSender EventLoop::MakeRawSender(const Channel *channel);
// Will run callback each time block of data received.
void EventLoop::MakeRawWatcher(const Channel *channel,
							   std::function<void(const uint8_t*)> callback);
// Returns raw message for a given channel.
RawFetcher EventLoop::MakeRawFetcher(const Channel *channel);
// Generate JSON from flatbuffer based on schema.
[channel](const uint8_t* data) {
	std::cout << FlatbufferToJson(channel->schema(), data) << std::endl;
}
```



## Fetcher Optimization

```cpp
// Non-fetcher example (slow):
//
// Register callback for goal.
void SaveGoal(const Goal &g) { goal_ = g; }
// Run whenever sensor data comes in.
void DoControls(const Position &p) {
	PID (p, goal_);
}

// Optimization the fetcher allows:
//
// Just get latest goal and process it.
void DoControls(const Position &p) {
	goal_fetcher_.Fetch();
	PID (p, goal_fetcher_.get());
}
```