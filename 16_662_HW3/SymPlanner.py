from typing import List, Tuple, Dict, Optional, Iterable, Set
from dataclasses import dataclass, field
import heapq

Predicate = Tuple[str, Tuple[str, ...]]   # e.g., ('at', ('robot','A'))
Subst = Dict[str, str]

def is_var(x: str) -> bool:
    return isinstance(x, str) and x.startswith("?")

def substitute(pred: Predicate, subst: Subst) -> Predicate:
    name, args = pred
    return (name, tuple(subst.get(a, a) for a in args))

def unify(a: Predicate, b: Predicate, subst: Optional[Subst]=None) -> Optional[Subst]:
    """Unify predicate a (may contain vars) with ground predicate b (state fact)."""
    if subst is None:
        subst = {}
    if a[0] != b[0] or len(a[1]) != len(b[1]):
        return None
    theta = dict(subst)
    for av, bv in zip(a[1], b[1]):
        if is_var(av):
            if av in theta and theta[av] != bv:
                return None
            theta[av] = bv
        else:
            if av != bv:
                return None
    return theta

def unify_all(preconds: List[Predicate], state: Set[Predicate]) -> List[Subst]:
    """All substitutions that satisfy all preconditions in the given ground state."""
    sols = []
    preconds = list(preconds)

    def backtrack(i: int, theta: Subst):
        if i == len(preconds):
            sols.append(theta.copy())
            return
        p = preconds[i]
        for s in state:
            if s[0] == p[0] and len(s[1]) == len(p[1]):
                new_theta = unify(substitute(p, theta), s, theta)
                if new_theta is not None:
                    backtrack(i + 1, new_theta)

    backtrack(0, {})
    # Deduplicate
    uniq, seen = [], set()
    for th in sols:
        key = tuple(sorted(th.items()))
        if key not in seen:
            seen.add(key)
            uniq.append(th)
    return uniq

@dataclass(frozen=True)
class ActionSchema:
    name: str
    parameters: Tuple[str, ...]                 # variable names like ?x
    preconds: Tuple[Predicate, ...]             # positive literals only (STRIPS)
    add_effects: Tuple[Predicate, ...]
    del_effects: Tuple[Predicate, ...] = field(default_factory=tuple)

    def ground(self, subst: Subst):
        return GroundAction(
            name=self.name,
            args=tuple(subst.get(v, v) for v in self.parameters),
            add=tuple(substitute(p, subst) for p in self.add_effects),
            delete=tuple(substitute(p, subst) for p in self.del_effects)
        )

@dataclass(frozen=True)
class GroundAction:
    name: str
    args: Tuple[str, ...]
    add: Tuple[Predicate, ...]
    delete: Tuple[Predicate, ...]
    def label(self) -> str:
        return f"{self.name}({', '.join(self.args)})"

class Planner:
    def __init__(self, actions: List[ActionSchema]):
        self.actions = actions

    def applicable_actions(self, state: Set[Predicate]) -> List[GroundAction]:
        out = []
        for schema in self.actions:
            for theta in unify_all(list(schema.preconds), state):
                out.append(schema.ground(theta))
        return out

    def apply(self, state: Set[Predicate], action: GroundAction) -> Set[Predicate]:
        new_state = set(state)
        for d in action.delete:
            if d in new_state:
                new_state.remove(d)
        for a in action.add:
            new_state.add(a)
        return new_state

    def heuristic(self, state: Set[Predicate], goal: Set[Predicate]) -> int:
        # Simple goal-count heuristic
        return sum(1 for g in goal if g not in state)

    def plan(self, init: Iterable[Predicate], goal: Iterable[Predicate], max_expansions: int = 50000):
        init_state = frozenset(init)
        goal_set = set(goal)

        # A* over state space
        frontier = []
        g_cost = {init_state: 0}
        parent = {init_state: (None, None)}  # state -> (prev_state, action)
        h0 = self.heuristic(set(init_state), goal_set)
        heapq.heappush(frontier, (h0, 0, init_state))

        expansions = 0
        while frontier and expansions < max_expansions:
            f, gc, state = heapq.heappop(frontier)
            expansions += 1

            if all(g in state for g in goal_set):
                # Reconstruct plan
                seq, s = [], state
                while parent[s][0] is not None:
                    prev, act = parent[s]
                    seq.append(act)
                    s = prev
                seq.reverse()
                return seq  # list[GroundAction]

            for act in self.applicable_actions(set(state)):
                new_state = frozenset(self.apply(set(state), act))
                new_g = gc + 1
                if new_state not in g_cost or new_g < g_cost[new_state]:
                    g_cost[new_state] = new_g
                    parent[new_state] = (state, act)
                    h = self.heuristic(set(new_state), goal_set)
                    heapq.heappush(frontier, (new_g + h, new_g, new_state))

        return None  # no plan found within limits
