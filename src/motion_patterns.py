"""Registered Studio angle commands (degrees, joint indices 1..4, seconds)."""
import ast
from copy import deepcopy
from functools import lru_cache
import math
from pathlib import Path
import re

import numpy as np
import yaml


@lru_cache(maxsize=1)
def patterns():
    path = Path(__file__).resolve().parents[1] / 'config/motion_patterns.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def apply_pattern(control, name):
    if name not in patterns():
        raise ValueError(f'Unknown motion pattern: {name}; use --list-patterns')
    pattern = deepcopy(patterns()[name])
    control.pop('expression_repeat_cycles', None)
    control.update(mode='expressions', pattern=name, resolved_pattern=name, expressions=pattern['expressions'],
                   pattern_name=pattern['name'], pattern_source=pattern.get('source'),
                   expression_period=pattern['period'],
                   expression_start_delay=pattern.get('start_delay', 0.))


def clear_pattern(control):
    for key in ('pattern', 'resolved_pattern', 'expressions', 'pattern_name', 'pattern_source',
                'expression_period', 'expression_start_delay', 'expression_repeat_cycles'):
        control.pop(key, None)


_BASE = dict(pi=math.pi, sin=math.sin, cos=math.cos, floor=math.floor, min=min, max=max)
_NODES = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Add, ast.Sub, ast.Mult,
          ast.Div, ast.Mod, ast.USub, ast.UAdd, ast.Constant, ast.Name, ast.Load, ast.Call)


@lru_cache(maxsize=128)
def compile_expressions(expressions):
    """Compile only scalar arithmetic and calls to previously defined functions.

    No Python attributes, indexing, imports, builtins, or arbitrary code are allowed.
    Capturing each definition's environment also prevents recursive definitions.
    """
    environment = dict(_BASE)
    for line in expressions:
        match = re.fullmatch(r'\s*([A-Za-z]\w*)(?:\(([^()]*)\))?\s*=\s*(.+)', line)
        if not match:
            raise ValueError(f'Invalid motion expression: {line}')
        name, parameters, expression = match.groups()
        arguments = tuple(p.strip() for p in parameters.split(',')) if parameters is not None else ()
        if (any(not re.fullmatch(r'[A-Za-z]\w*', p) for p in arguments)
                or len(set(arguments)) != len(arguments) or name in environment):
            raise ValueError(f'Invalid or duplicate motion definition: {line}')
        try:
            tree = ast.parse(expression, mode='eval')
        except SyntaxError as error:
            raise ValueError(f'Invalid motion expression: {line}') from error
        for node in ast.walk(tree):
            if not isinstance(node, _NODES):
                raise ValueError(f'Unsupported motion expression: {line}')
            if isinstance(node, ast.Constant) and type(node.value) not in (int, float):
                raise ValueError('Motion constants must be numbers')
            if isinstance(node, ast.Name) and node.id not in environment and node.id not in arguments:
                raise ValueError(f'Unknown motion variable: {node.id}')
            if isinstance(node, ast.Call) and (not isinstance(node.func, ast.Name)
                    or not callable(environment.get(node.func.id)) or node.keywords):
                raise ValueError(f'Unsupported motion call: {line}')
        code = compile(tree, '<motion expression>', 'eval')
        scope = environment.copy()

        def evaluate(*values, _code=code, _scope=scope, _arguments=arguments):
            if len(values) != len(_arguments):
                raise ValueError('Wrong number of motion function arguments')
            return eval(_code, {'__builtins__': {}}, dict(_scope, **dict(zip(_arguments, values))))

        try:
            environment[name] = evaluate if parameters is not None else evaluate()
        except (ArithmeticError, TypeError) as error:
            raise ValueError(f'Invalid motion constant: {line}') from error
        if name == 'x' and arguments != ('n', 't'):
            raise ValueError('Motion output must be x(n,t)')
    output = environment.get('x')
    if not callable(output):
        raise ValueError('Motion expressions must define x(n,t)')
    return output


def expression_targets(t, control):
    output = compile_expressions(tuple(control['expressions']))
    if control.get('period') is not None:
        # Run one complete original cycle per common period, preserving phase
        # and the relative durations of each stroke, recovery and rest segment.
        t = (control.get('expression_start_delay', 0.)
             + t * control['expression_period'] / control['period'])
    try:
        degrees = np.array([output(n, t) for n in range(1, 5)], dtype=float)
    except (ArithmeticError, TypeError) as error:
        raise ValueError(f'Invalid motion target at t={t}: {error}') from error
    if degrees.shape != (4,) or not np.all(np.isfinite(degrees)):
        raise ValueError('Motion expressions must return four finite angles')
    return np.deg2rad(degrees)


