# Triage Labels

The skills speak in terms of five canonical triage roles. This file maps those roles to the actual label strings used in this repo's issue tracker.

| Label in mattpocock/skills | Label in our tracker | Meaning                                  |
| -------------------------- | -------------------- | ---------------------------------------- |
| `needs-triage`             | `needs-triage`       | Maintainer needs to evaluate this issue  |
| `needs-info`               | `needs-info`         | Waiting on reporter for more information |
| `ready-for-agent`          | `ready-for-agent`    | Fully specified, ready for an AFK agent  |
| `ready-for-human`          | `ready-for-human`    | Requires human implementation            |
| `wontfix`                  | `wontfix`            | Will not be actioned                     |

When a skill mentions a role (e.g. "apply the AFK-ready triage label"), use the corresponding label string from this table.

Edit the right-hand column to match whatever vocabulary you actually use.

## Closing an issue drops its triage label

Four of these five labels answer one question: **what does this issue need next?**
A closed issue needs nothing next, so the answer is no label rather than a label
meaning "nothing" — remove `needs-triage`, `needs-info`, `ready-for-agent` or
`ready-for-human` as part of closing.

There is no `closed` label and should not be. GitHub already records open/closed
as state, and a label saying the same thing is a second copy that can disagree
with the first the moment somebody reopens the issue. Ask the tracker.

`wontfix` is the exception and stays. It does not say what the issue needs next;
it says why the issue closed, which is a fact GitHub records nowhere else.

`wayfinder:*` labels also stay. They are not triage states — they say what kind
of work a child ticket is, and that remains true after it is done.
