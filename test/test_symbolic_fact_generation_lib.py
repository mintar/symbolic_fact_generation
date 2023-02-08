import unittest

from symbolic_fact_generation.msg import Facts as FactsMsg
from symbolic_fact_generation.msg import Fact as FactMsg
from symbolic_fact_generation.msg import ChangedFacts as ChangedFactsMsg
from symbolic_fact_generation.common.fact import Fact


class TestSymbolicFactGenerationLib(unittest.TestCase):
    def test_convert_facts_to_msg(self):
        from symbolic_fact_generation.common.lib import convert_facts_to_msg

        expected_msg = FactsMsg(facts=[FactMsg(name="on", values=["klt", "table"]), FactMsg(
            name="holding", values=["nothing"]), FactMsg(name="at", values=["lab"])])
        fact_list = [Fact(name="on", values=["klt", "table"]), Fact(
            name="holding", values=["nothing"]), Fact(name="at", values=["lab"])]

        self.assertEqual(convert_facts_to_msg(fact_list), expected_msg)

    def test_convert_changed_facts_to_msg(self):
        from symbolic_fact_generation.common.lib import convert_changed_facts_to_msg

        expected_msg = ChangedFactsMsg(update_types=[0, 1], changed_facts=[FactMsg(name="on", values=["klt", "table"]), FactMsg(
            name="holding", values=["nothing"]), FactMsg(name="at", values=["lab"])])
        fact_list = [Fact(name="on", values=["klt", "table"]), Fact(
            name="holding", values=["nothing"]), Fact(name="at", values=["lab"])]

        self.assertEqual(convert_changed_facts_to_msg([0, 1], fact_list), expected_msg)

    def test_split_object_class_from_id_with_normal_case(self):
        from symbolic_fact_generation.common.lib import split_object_class_from_id

        expected_result = ('class', 1)
        input = 'class_1'
        result = split_object_class_from_id(input)

        self.assertEqual(result, expected_result)

    def test_split_object_class_from_id_with_no_underscores(self):
        from symbolic_fact_generation.common.lib import split_object_class_from_id

        expected_result = ('class', None)
        input = 'class'
        result = split_object_class_from_id(input)

        self.assertEqual(result, expected_result)

    def test_split_object_class_from_id_with_no_id(self):
        from symbolic_fact_generation.common.lib import split_object_class_from_id

        expected_result = ('class_a', None)
        input = 'class_a'
        result = split_object_class_from_id(input)

        self.assertEqual(result, expected_result)


PKG = 'symbolic_fact_generation'
NAME = 'test_symbolic_fact_generation_lib'

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestSymbolicFactGenerationLib)
