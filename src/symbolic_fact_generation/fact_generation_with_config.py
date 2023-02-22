#!/usr/bin/python3

# BSD 3-Clause License

# Copyright (c) 2022, DFKI Niedersachsen
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import List

from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.lib import read_yaml_file
from symbolic_fact_generation.common.fact import Fact


class FactGenerationWithConfig(GeneratorInterface):
    def __init__(self, config_file_path: str = 'config/facts_config.yaml'):
        facts_config = read_yaml_file(config_file_path)

        # create a list of classes with generate_facts function to generate the specified facts from the config file
        self._fact_classes = []
        for fact in facts_config['facts']:
            try:
                fact_config = fact.popitem()

                # import fact generation module, class, params and
                # save (fact name, instance object of fact generator class) in list
                self._fact_classes.append((fact_config[0], getattr(__import__(fact_config[1]['module'], fromlist=[
                                          fact_config[1]['class']]), fact_config[1]['class'])(fact_config[0], *fact_config[1]['params'])))
            except ImportError as e:
                print(f"Could not import module for {fact_config[0]} fact: {e}")
            except AttributeError as e:
                print(f"Could not import class for {fact_config[0]} fact: {e}")
            except Exception as e:
                print(f"Error while processing fact config file: {e}")

        self._current_facts = []

    def generate_facts(self) -> List[Fact]:
        # list to save all generated facts
        generated_facts = []

        # iterate trough all fact generators
        for generator in self._fact_classes:
            try:
                # generate facts
                generated_facts.extend(generator[1].generate_facts())
            except AttributeError:
                print(
                    f"Specified generator class for fact {generator[0]} has no generate_facts() method! Could not generate {generator[0]} facts!")

        # compare generated facts with current facts to add and remove facts to the current facts
        for fact in list(self._current_facts):
            if fact in generated_facts:
                generated_facts.remove(fact)
            else:
                self._current_facts.remove(fact)
        self._current_facts.extend(generated_facts)

        return self._current_facts

    def generate_facts_with_name(self, fact_name: str) -> List[Fact]:
        # list to save all generated facts
        generated_facts = []

        # iterate trough all fact generators
        for generator in self._fact_classes:
            try:
                # generate only facts with the given fact name
                if fact_name in generator[0]:
                    generated_facts.extend(generator[1].generate_facts())
            except AttributeError:
                print(
                    f"Specified generator class for fact {generator[0]} has no generate_facts() method! Could not generate {generator[0]} facts!")
        
        # make a copy of the generated facts to return later
        new_facts = list(generated_facts)

        # compare generated facts with current facts to add and remove facts to the current facts
        for fact in list(self._current_facts):
            if fact in generated_facts:
                generated_facts.remove(fact)
            else:
                self._current_facts.remove(fact)
        self._current_facts.extend(generated_facts)

        return new_facts
