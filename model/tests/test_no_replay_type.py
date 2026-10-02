"""Type 3 executes like type 0 in the model, which does not model replay."""
import random
import unittest

from model.bingo_sim import BingoSimulator, SimConfig
from model.bingo_sim_chiplet import ChipletModel, DoneInfo, EXEC_TASK_TYPES, TaskDescriptor


def task(kind, dep_set=False, core=0, task_id=7, dep_check=False):
    return TaskDescriptor(
        task_type=kind, task_id=task_id, assigned_chiplet_id=0,
        assigned_cluster_id=0, assigned_core_id=core,
        dep_check_en=dep_check, dep_check_code=1 if dep_check else 0,
        dep_set_en=dep_set, dep_set_all_chiplet=False,
        dep_set_chiplet_id=0, dep_set_cluster_id=0,
        dep_set_code=2 if dep_set else 0)


def simulator():
    return BingoSimulator(SimConfig(
        num_chiplets=1, num_clusters_per_chiplet=1, num_cores_per_cluster=2,
        work_delay_range=(5, 5), random_seed=0))


class NoReplayTypeTests(unittest.TestCase):
    def test_single_task_must_complete_with_same_latency(self):
        results = []
        for kind in (0, 3):
            sim = simulator()
            sim.load_tasks({0: [task(kind)]})
            self.assertEqual(sim._all_task_ids, {7})
            result = sim.run(max_cycles=100)
            self.assertFalse(result.deadlock_detected)
            self.assertEqual(result.completed_task_ids, {7})
            self.assertEqual(result.total_latency, 8)
            results.append(result.total_latency)
        self.assertEqual(results[0], results[1])

    def test_checkout_waits_for_own_done_in_all_twelve_cases(self):
        for kind in EXEC_TASK_TYPES:
            for dep_set in (False, True):
                for mode in ("single", "per_core"):
                    with self.subTest(kind=kind, dep_set=dep_set, mode=mode):
                        model = ChipletModel(0, 1, 2, random.Random(0), done_queue_mode=mode)
                        checkout, done = model.checkout_queues[0][0], model.done_queues[0]
                        self.assertTrue(checkout.push(task(kind, dep_set)))
                        self.assertIsNone(model._try_checkout_dep_set(0, 0, 0))
                        self.assertEqual(checkout.count, 1)
                        # An unrelated slot's done cannot release this head.
                        self.assertTrue(done.push(DoneInfo(99, 1, 1)))
                        self.assertIsNone(model._try_checkout_dep_set(0, 0, 1))
                        self.assertEqual((checkout.count, done.count), (1, 1))
                        done.pop()
                        self.assertTrue(done.push(DoneInfo(7, 0, 0)))
                        events = model._try_checkout_dep_set(0, 0, 2)
                        self.assertTrue(checkout.empty)
                        self.assertTrue(done.empty)
                        self.assertEqual([e.event_type for e in events],
                                         ["DEP_SET"] if dep_set else [])

    def test_chain_dispatch_and_done_times_are_identical(self):
        results = []
        for kind in (0, 3):
            sim = simulator()
            sim.load_tasks({0: [task(kind, True), task(0, core=1, task_id=8, dep_check=True)]})
            result = sim.run(max_cycles=100)
            self.assertEqual(result.completed_task_ids, {7, 8})
            self.assertFalse(result.deadlock_detected)
            events = [(e.task_id, e.event_type, e.time) for e in result.trace.events
                      if e.event_type in ("TASK_DISPATCHED", "TASK_DONE")]
            times = {(n, event): time for n, event, time in events}
            self.assertGreater(times[(8, "TASK_DISPATCHED")], times[(7, "TASK_DONE")])
            results.append((events, result.total_latency))
        self.assertEqual(results[0], results[1])

    def test_out_of_range_type_is_not_an_executing_entry(self):
        self.assertEqual(EXEC_TASK_TYPES, (0, 2, 3))
        sim = simulator()
        sim.load_tasks({0: [task(4)]})
        self.assertEqual(sim._all_task_ids, set())
        model = ChipletModel(0, 1, 2, random.Random(0))
        model.checkout_queues[0][0].push(task(4))
        model.done_queues[0].push(DoneInfo(7, 0, 0))
        self.assertIsNone(model._try_checkout_dep_set(0, 0, 0))
        self.assertEqual(model.checkout_queues[0][0].count, 1)


if __name__ == "__main__":
    unittest.main()
