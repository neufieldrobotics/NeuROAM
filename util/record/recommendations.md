Here’s your text converted to Markdown:

---

# Tuning Zenoh for Dropped Messages

When you're dropping messages, the most likely culprits are **network congestion**, **insufficient buffer sizes**, or **overly aggressive timeouts**—especially when dealing with bursty traffic or a large number of nodes.

You can tune parameters by creating your own **JSON5 configuration files** and setting the `ZENOH_ROUTER_CONFIG_URI` and `ZENOH_SESSION_CONFIG_URI` environment variables. For quick tests, the `ZENOH_CONFIG_OVERRIDE` environment variable is very powerful.

Here are the key parameters I recommend you investigate, focusing on the **session configuration** (`DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`) first, as it affects each node directly.

---

## Key Parameters to Tune for Dropped Messages

### 1. Congestion Control

This is the most critical area. When a communication link is congested, Zenoh has to decide whether to block (wait for the queue to clear) or drop messages.

* **`transport.link.tx.queue.congestion_control.block.wait_before_close`**

  * **What it is:** The time (in microseconds) to wait when a queue is full before closing the connection.
  * **Default (Session):** `60000000` (60 seconds)
  * **Default (Router):** `5000000` (5 seconds)
  * **Recommendation:** The session default is already very high, which is good—it prevents nodes from being disconnected during startup traffic storms. The router's default is lower. If your nodes communicate across different machines, you might experience drops if the router is congested. Try increasing the router's `wait_before_close` value.

---

### 2. Transmission Queue Size

If you have topics that publish large amounts of data in bursts, the default queue sizes might not be enough to handle them.

* **`transport.link.tx.queue.size`**

  * **What it is:** The number of batches each priority queue can hold. There are **8 priority levels** (`real_time`, `data_high`, `data`, etc.).
  * **Default:** `2` for all priority levels.
  * **Recommendation:** This is quite small. For topics that are dropping messages, try increasing the queue size for the relevant priority level (e.g., `data_high` or `data`). Try increasing this to **4 or 8** in your session config.

---

### 3. Receive Buffer Size

The receiver also needs buffers to store incoming data.

* **`transport.link.rx.buffer_size`**

  * **What it is:** The size (in bytes) of the receiving buffer for each link.
  * **Default:** `65535` bytes.
  * **Recommendation:** The comments note that for *very high throughput scenarios*, the `rx_buffer_size` can be increased. If you are sending large messages (like images or point clouds), you should definitely increase this value. Try doubling it to **131070**.

---

## How to Apply Changes (Example)

Let's say you want to increase the queue size for the `data` and `data_high` priorities to **4** for your ROS nodes. You can use the `ZENOH_CONFIG_OVERRIDE` variable without creating a new file:

```bash
export ZENOH_CONFIG_OVERRIDE='transport/link/tx/queue/size/data=4;transport/link/tx/queue/size/data_high=4'

# Now run your ROS 2 nodes with:
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

---

**Recommendation:**
Start with the **Congestion Control** and **Queue Size** parameters in your **session configuration**. Monitor your topics and see if the message drop rate improves. If the problem persists—especially with multi-machine setups—then start tuning the **router configuration** as well.

---

If you want, I can also create a **clean Markdown table** for these parameters so it’s easier to compare defaults and recommendations. That would make this much more scannable. Would you like me to do that?
