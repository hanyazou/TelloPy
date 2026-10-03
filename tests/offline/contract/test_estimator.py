"""Estimator: a Container whose Samples are derived from those of other Containers."""
import unittest

from tellopy import Container, Estimator, Sample


class Doubling(Estimator):
    """An Estimator of the smallest kind: for each Sample of its input, one of its own."""

    def __init__(self, source):
        super(Doubling, self).__init__()
        self._listen(source, self._on_sample)

    def _on_sample(self, sample):
        self.add(Sample(event_time=sample.event_time * 2))


class EstimatorTest(unittest.TestCase):

    def test_what_it_works_out_comes_out_as_the_samples_of_a_container(self):
        source = Container()
        estimator = Doubling(source)
        heard = []
        estimator.subscribe(heard.append)
        for t in (1.0, 2.0, 3.0):
            source.add(Sample(event_time=t))
        self.assertIsInstance(estimator, Container)
        self.assertEqual([s.event_time for s in estimator], [2.0, 4.0, 6.0])
        self.assertEqual([s.event_time for s in heard], [2.0, 4.0, 6.0])
        self.assertEqual(estimator.latest().event_time, 6.0)

    def test_it_has_a_name_like_any_container(self):
        self.assertEqual(Doubling(Container()).name, 'Doubling')
        self.assertEqual(Estimator(name='mine').name, 'mine')

    def test_closing_stops_it_listening_to_its_inputs(self):
        source = Container()
        estimator = Doubling(source)
        source.add(Sample(event_time=1.0))
        estimator.close()
        source.add(Sample(event_time=2.0))
        self.assertEqual([s.event_time for s in estimator], [2.0])
