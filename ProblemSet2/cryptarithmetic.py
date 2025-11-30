from typing import Tuple
import re
from CSP import Assignment, Problem, UnaryConstraint, BinaryConstraint

#TODO (Optional): Import any builtin library or define any helper function you want to use

# This is a class to define for cryptarithmetic puzzles as CSPs
class CryptArithmeticProblem(Problem):
    LHS: Tuple[str, str]
    RHS: str

    # Convert an assignment into a string (so that is can be printed).
    def format_assignment(self, assignment: Assignment) -> str:
        LHS0, LHS1 = self.LHS
        RHS = self.RHS
        letters = set(LHS0 + LHS1 + RHS)
        formula = f"{LHS0} + {LHS1} = {RHS}"
        postfix = []
        valid_values = list(range(10))
        for letter in letters:
            value = assignment.get(letter)
            if value is None: continue
            if value not in valid_values:
                postfix.append(f"{letter}={value}")
            else:
                formula = formula.replace(letter, str(value))
        if postfix:
            formula = formula + " (" + ", ".join(postfix) +  ")" 
        return formula

    @staticmethod
    def from_text(text: str) -> 'CryptArithmeticProblem':
  # Given a text in the format "LHS0 + LHS1 = RHS", the following regex
        # matches and extracts LHS0, LHS1 & RHS
        # For example, it would parse "SEND + MORE = MONEY" and extract the
        # terms such that LHS0 = "SEND", LHS1 = "MORE" and RHS = "MONEY"
        pattern = r"\s*([a-zA-Z]+)\s*\+\s*([a-zA-Z]+)\s*=\s*([a-zA-Z]+)\s*"
        match = re.match(pattern, text)
        if not match: raise Exception("Failed to parse:" + text)
        LHS0, LHS1, RHS = [match.group(i+1).upper() for i in range(3)]

        problem = CryptArithmeticProblem()
        problem.LHS = (LHS0, LHS1)
        problem.RHS = RHS

        #TODO Edit and complete the rest of this function
        # problem.variables:    should contain a list of variables where each variable is string (the variable name)
        # problem.domains:      should be dictionary that maps each variable (str) to its domain (set of values)
        #                       For the letters, the domain can only contain integers in the range [0,9].
        # problem.constaints:   should contain a list of constraint (either unary or binary constraints).


        # Extract the operands and result
        term1, term2 = problem.LHS
        term3 = problem.RHS

        # Collect all unique letter variables from the three words
        problem.variables = list(set(term1) | set(term2) | set(term3))
        
        # each letter can be assigned a digit from 0 to 9
        problem.domains = {v: set(range(10)) for v in problem.variables}
        
        # leading letters canot be 0 
        problem.constraints = [
            UnaryConstraint(term1[0], lambda x: x != 0),
            UnaryConstraint(term2[0], lambda x: x != 0),
            UnaryConstraint(term3[0], lambda x: x != 0),
        ]

        # all letters must have different vales
        for idx, var1 in enumerate(problem.variables):
            for var2 in problem.variables[idx + 1:]:
                problem.constraints.append(BinaryConstraint((var1, var2), lambda a, b: a != b))

        # Create carry variables for each column (C0, C1, C2, ...)
        max_size = max(len(LHS0), len(LHS1))
        flags = [f"C{k}" for k in range(max_size + 1)]

        problem.variables.extend(flags)

        # Carry can only be 0 or 1
        for f in flags:
            problem.domains[f] = {0, 1}

        # the first carry is always 0
        problem.constraints.append(UnaryConstraint("C0", lambda x: x == 0))

        # Process columns where both operands have values
        min_size = min(len(LHS0), len(LHS1))

        for pos in range(1, min_size + 1):
            # Get the digits at this position 
            digit1 = LHS0[-pos]
            digit2 = LHS1[-pos]
            flag = f"C{pos-1}"

            # Create a composite variable representing the three values
            comp = f"{digit1}{digit2}{flag}"
            problem.variables.append(comp)

            # Domain for composite variable
            problem.domains[comp] = {
                f"{x}{y}{z}" for x in range(10) for y in range(10) for z in range(2)
            }

            #first character of composite matches digit1
            problem.constraints.append(BinaryConstraint((comp, digit1), lambda s, a: s[0] == str(a)))
            #second character of composite matches digit2
            problem.constraints.append(BinaryConstraint((comp, digit2), lambda s, b: s[1] == str(b)))
            # third character of composite matches carry flag
            problem.constraints.append(BinaryConstraint((comp, flag), lambda s, c: s[2] == str(c)))

            # get result digit
            problem.constraints.append(BinaryConstraint((comp, RHS[-pos]), 
                lambda text, result: sum(int(text[i]) for i in range(len(text))) % 10 == result))
            #get carry_out
            problem.constraints.append(BinaryConstraint((comp, f"C{pos}"), 
                lambda text, result: sum(int(text[i]) for i in range(len(text))) // 10 == result))

        # Process remaining columns when only one digit
        if len(LHS0) != len(LHS1):
            longer = LHS0 if len(LHS0) > len(LHS1) else LHS1
            start = min_size
            end = len(longer)

            for pos in range(start + 1, end + 1):
                # Get the digit from the longer operand
                digit = longer[-pos]
                flag = f"C{pos-1}"
                comp = f"{digit}{flag}"

                problem.variables.append(comp)
                # Domain for two-element composite: digit and carry
                problem.domains[comp] = {f"{x}{c}" for x in range(10) for c in range(2)}

                # Constraint: first character matches the digit
                problem.constraints.append(BinaryConstraint((comp, digit), lambda s, a: s[0] == str(a)))
                # Constraint: second character matches the carry
                problem.constraints.append(BinaryConstraint((comp, flag), lambda s, c: s[1] == str(c)))

                # Sum constraint: (digit + carry_in) mod 10 equals result digit
                problem.constraints.append(BinaryConstraint((comp, RHS[-pos]), 
                    lambda text, result: sum(int(text[i]) for i in range(len(text))) % 10 == result))
                # Carry constraint: (digit + carry_in) // 10 equals carry_out
                problem.constraints.append(BinaryConstraint((comp, f"C{pos}"), 
                    lambda text, result: sum(int(text[i]) for i in range(len(text))) // 10 == result))

        # Handle last carry
        if len(RHS) > max_size:
            # Result has more digits leftmost digit must be 1 and final carry must be 1
            problem.constraints.append(UnaryConstraint(RHS[0], lambda x: x == 1))
            problem.constraints.append(UnaryConstraint(f"C{max_size}", lambda x: x == 1))
        elif len(RHS) == max_size:
            # Result has same digits no overflow
            problem.constraints.append(UnaryConstraint(f"C{max_size}", lambda x: x == 0))

        return problem


    # Read a cryptarithmetic puzzle from a file
    @staticmethod
    def from_file(path: str) -> "CryptArithmeticProblem":
        with open(path, 'r') as f:
            return CryptArithmeticProblem.from_text(f.read())
