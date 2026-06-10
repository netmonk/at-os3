# Kernel FSM
# Constitutional Contract — Multi-Architecture Edition

Copyright (C) 2026 Dominique CARREL (netmonk) <netmonk@netmonk.org>
SPDX-License-Identifier: CC-BY-ND-4.0

Licensed under Creative Commons Attribution-NoDerivatives 4.0 International.

> This kernel preserves causal truth across architectures.  
> Hardware may change.  
> Causality must not.

This constitution defines structural invariants that must hold on any supported architecture.

---

# PART I — PURPOSE

## Article 1 — Purpose

The sole purpose of this kernel is to preserve **causal integrity in time**.

The kernel is not:

- a resource allocator
- a throughput maximizer
- a fairness engine
- a scheduler

It is a deterministic event instrument.

Latency is a truth signal.  
Latency without causal clarity is invalid.

---

# PART II — TEMPORAL PRIMACY

## Article 2 — Event Time Model

All execution is driven exclusively by events.

There is no:

- background work
- implicit progression
- scheduler-driven execution
- time slicing
- fairness arbitration

If progress occurs without an event, the design is invalid.

---

## Article 2.1 — Optimal Grain Requirement

Each subsystem must operate at the abstraction scale that maximizes **causal effectiveness**.

If micro-fragmentation introduces ambiguity, hidden branching, or indistinguishable convergence, a macro-level abstraction is constitutionally required.

Macro abstractions are valid only if they:

- increase determinism
- reduce degeneracy
- preserve explicit transitions

---

# PART III — CAUSAL EFFECTIVENESS

## Article 3 — Definition

Causal Effectiveness = Determinism − Degeneracy

Where:

- Determinism = consistency of effect given cause
- Degeneracy = multiple distinct causes collapsing into indistinguishable outcomes

The kernel must maximize causal effectiveness across scales.

---

## Article 3.1 — Distinguishability

Distinct causes must remain distinguishable within system representation.

Forbidden patterns:

- Multiple errors → single opaque error
- Multiple commands → monolithic dispatch blob
- Multiple deadlines → indistinguishable timer event
- Multiple states → implicit flag combinations
- Multiple transitions → hidden branching logic

If convergence is necessary, it must be explicit and justified.

---

# PART IV — EVENT SUPREMACY

## Article 4 — Canonical Event Pipeline

All control flow originates from events.

Canonical pipeline:

ISR → event_enqueue → event_loop → dispatch → fsm_step / handler

No fast paths.  
No direct invocation bypassing the queue.  
No exceptional execution channels.

If a shortcut exists, it will be used.  
Therefore, it must not exist.

---

# PART V — INTERRUPT MINIMALISM

## Article 5 — ISR Discipline

ISR context may:

- Acknowledge or clear hardware source
- Capture minimal immutable data
- Enqueue exactly one event

ISR context may NOT:

- Allocate memory
- Block or spin
- Execute protocol logic
- Perform unbounded loops
- Emit heavy logging
- Modify FSM state

ISR must be mechanically auditable and structurally bounded.

Interrupts preserve time.  
They do not interpret it.

---

# PART VI — EXECUTION TAXONOMY

## Article 6 — FSM

A module qualifies as FSM only if:

- It has explicit `state`
- Progression is table-driven:
  table[state][event] → (next_state, action)
- `fsm_step` is sole authority for state mutation
- Actions do NOT modify state directly
- Events emitted by actions trigger future steps only

No implicit transitions.  
No state writes outside `fsm_step`.

---

## Article 7 — Handler

A handler:

- Reacts to a single event
- Performs bounded logic
- Has no progression semantics
- Has no state transitions

Handlers are reactions, not processes.

---

## Article 8 — Service

A service:

- Provides infrastructure capability
- Has no autonomous behavior
- Does not introduce scheduling
- Does not create implicit progression

Services enable; they do not decide.

---

# PART VII — NON-BYPASSABILITY

## Article 9 — Structural Enforcement

All invariants must be enforced structurally.

Debug bypasses, temporary shortcuts, and conditional escape hatches are forbidden.

If correctness relies on developer discipline rather than structure, the architecture is invalid.

---

# PART VIII — LAYERED SOVEREIGNTY

## Article 10 — Layer Roles

Layers are defined by responsibility, not hardware:

- `core/` — event engine, FSM engine, canonical services
- `drivers/` — hardware abstraction only
- `subfsm/` — domain FSMs
- `handlers/` — bounded event reactions
- `ports/<arch>/` — architecture-specific glue

Constraints:

- Drivers must not contain domain logic
- Core must not contain hardware addresses
- Subfsm must not access hardware directly
- Ports must not alter event semantics

Dependencies are strictly unidirectional.

---

# PART IX — ABI SANCTITY

## Article 11 — Boundary Contracts

Every cross-layer call defines an ABI.

ABIs must be:

- Explicit
- Minimal
- Stable
- Architecture-neutral (in core)

No structure may be accessed across boundaries without explicit contract.

---

## Article 11.1 — Calling Convention Compliance

Each supported architecture must define:

- Register preservation rules
- Stack alignment requirements
- Call boundary obligations

The kernel must strictly obey the active architecture ABI.

Violation = structural corruption.

---

# PART X — MULTI-ARCH PORTABILITY

## Article 12 — Architecture Isolation

The core kernel must not contain:

- MMIO addresses
- IRQ numbers
- Clock constants
- Fixed memory maps
- SoC-specific assumptions

All hardware details must reside in:

ports/<arch>/
drivers/<arch>/

Core logic must remain hardware-agnostic.

---

## Article 12.1 — Port Proof Obligation

Each architecture port must prove:

- ISR minimalism preserved
- Canonical event pipeline unchanged
- ABI compliance enforced
- Deterministic timer semantics preserved
- No hidden scheduling introduced

---

# PART XI — TEMPORAL COHERENCE

## Article 13 — Timer Discipline

The timer subsystem must:

- Preserve absolute time ordering
- Avoid hidden reordering
- Avoid implicit background progression

Hierarchical or macro-timer abstractions are valid only if they increase causal effectiveness.

Timer abstraction must not introduce degeneracy.

---

# PART XII — PROOF OBLIGATIONS

Any change must satisfy all applicable proof obligations.

---

### PO-1 — Event Origin Proof
Every behavior originates from an explicit event.

### PO-2 — Canonical Path Proof
No alternate execution path exists.

### PO-3 — FSM Ownership Proof
Each action belongs to exactly one FSM.

### PO-4 — ISR Safety Proof
ISR execution time is bounded and structurally constrained.

### PO-5 — Layer Integrity Proof
No cross-layer leakage.

### PO-6 — Non-Bypassability Proof
No hidden control or data paths exist.

### PO-7 — Temporal Coherence Proof
Causal ordering is preserved across transitions.

### PO-8 — Degeneracy Audit Proof
Distinct causes remain distinguishable.
Convergence is explicit and justified.

---

# PART XIII — CONSEQUENCE

Correct behavior built on a structure that allows ambiguity is incorrect by definition.

Hardware may change.  
Clock frequencies may change.  
Interrupt controllers may change.  

Causality must not change.

---

# PART XIV — AXIOM

Micro correctness without macro clarity is incomplete.  
Macro elegance without micro rigor is false.

The valid design is the one that preserves causal structure at the scale where it is most effective.
