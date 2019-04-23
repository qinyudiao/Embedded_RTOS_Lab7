# This plots an event log captured from the OS

import matplotlib.pyplot as plt
import numpy as np
import re
import string

example = """\
Name: Interpreter  Time: 448108831 cycles  Type: FG START
Name: DAS  Time: 448123619 cycles  Type: PT START
Name: DAS  Time: 448124049 cycles  Type: PT END
Name: DAS  Time: 448163619 cycles  Type: PT START
Name: DAS  Time: 448164049 cycles  Type: PT END
Name: DAS  Time: 448203619 cycles  Type: PT START
Name: DAS  Time: 448204051 cycles  Type: PT END
Name: DAS  Time: 448243619 cycles  Type: PT START
Name: DAS  Time: 448244049 cycles  Type: PT END
Name: PID  Time: 448271889 cycles  Type: FG START
Name: IdleTask  Time: 448272399 cycles  Type: FG START
Name: DAS  Time: 448283619 cycles  Type: PT START
Name: DAS  Time: 448284049 cycles  Type: PT END
Name: ADC  Time: 448287155 cycles  Type: PT START
Name: PID  Time: 448287941 cycles  Type: FG START
Name: ADC  Time: 448288321 cycles  Type: PT END
Name: IdleTask  Time: 448288725 cycles  Type: FG START
Name: IdleTask  Time: 448294681 cycles  Type: FG START
Name: Display  Time: 448295563 cycles  Type: FG START
Name: IdleTask  Time: 448296011 cycles  Type: FG START
Name: Consumer  Time: 448296523 cycles  Type: FG START
Name: IdleTask  Time: 448297381 cycles  Type: FG START
Name: DAS  Time: 448323619 cycles  Type: PT START
Name: DAS  Time: 448324051 cycles  Type: PT END
Name: DAS  Time: 448363619 cycles  Type: PT START
Name: DAS  Time: 448364049 cycles  Type: PT END
Name: DAS  Time: 448403619 cycles  Type: PT START
Name: DAS  Time: 448404049 cycles  Type: PT END
Name: DAS  Time: 448443619 cycles  Type: PT START
Name: DAS  Time: 448444051 cycles  Type: PT END
Name: Display  Time: 448460653 cycles  Type: FG START
Name: IdleTask  Time: 448461099 cycles  Type: FG START
Name: DAS  Time: 448483619 cycles  Type: PT START
Name: DAS  Time: 448484049 cycles  Type: PT END
Name: ADC  Time: 448487513 cycles  Type: PT START
Name: ADC  Time: 448488187 cycles  Type: PT END
Name: DAS  Time: 448523619 cycles  Type: PT START
Name: DAS  Time: 448524049 cycles  Type: PT END
Name: DAS  Time: 448563711 cycles  Type: PT START
Name: DAS  Time: 448564143 cycles  Type: PT END
Name: DAS  Time: 448603621 cycles  Type: PT START
Name: DAS  Time: 448604051 cycles  Type: PT END
Name: Display  Time: 448624371 cycles  Type: FG START
Name: IdleTask  Time: 448624817 cycles  Type: FG START
Name: DAS  Time: 448643619 cycles  Type: PT START
Name: DAS  Time: 448644049 cycles  Type: PT END
Name: DAS  Time: 448683619 cycles  Type: PT START
Name: DAS  Time: 448684051 cycles  Type: PT END
Name: ADC  Time: 448687155 cycles  Type: PT START
Name: ADC  Time: 448687829 cycles  Type: PT END
Name: DAS  Time: 448723619 cycles  Type: PT START
Name: DAS  Time: 448724049 cycles  Type: PT END
Name: DAS  Time: 448763675 cycles  Type: PT START
Name: DAS  Time: 448764105 cycles  Type: PT END
Name: Display  Time: 448788087 cycles  Type: FG START
Name: IdleTask  Time: 448788533 cycles  Type: FG START
Name: DAS  Time: 448803621 cycles  Type: PT START
Name: DAS  Time: 448804053 cycles  Type: PT END
Name: DAS  Time: 448843619 cycles  Type: PT START
Name: DAS  Time: 448844049 cycles  Type: PT END
Name: DAS  Time: 448883619 cycles  Type: PT START
Name: DAS  Time: 448884049 cycles  Type: PT END
Name: ADC  Time: 448887475 cycles  Type: PT START
Name: ADC  Time: 448888149 cycles  Type: PT END
Name: DAS  Time: 448923619 cycles  Type: PT START
Name: DAS  Time: 448924051 cycles  Type: PT END
Name: Display  Time: 448951809 cycles  Type: FG START
Name: IdleTask  Time: 448952255 cycles  Type: FG START
Name: DAS  Time: 448963619 cycles  Type: PT START
Name: DAS  Time: 448964049 cycles  Type: PT END
Name: DAS  Time: 449003619 cycles  Type: PT START
Name: DAS  Time: 449004049 cycles  Type: PT END
Name: DAS  Time: 449043619 cycles  Type: PT START
Name: DAS  Time: 449044051 cycles  Type: PT END
Name: DAS  Time: 449083619 cycles  Type: PT START
Name: DAS  Time: 449084049 cycles  Type: PT END
Name: ADC  Time: 449087155 cycles  Type: PT START
Name: ADC  Time: 449087829 cycles  Type: PT END
Name: Display  Time: 449115525 cycles  Type: FG START
Name: IdleTask  Time: 449115971 cycles  Type: FG START
Name: DAS  Time: 449123619 cycles  Type: PT START
Name: DAS  Time: 449124049 cycles  Type: PT END
Name: DAS  Time: 449163655 cycles  Type: PT START
Name: DAS  Time: 449164087 cycles  Type: PT END
Name: DAS  Time: 449203621 cycles  Type: PT START
Name: DAS  Time: 449204051 cycles  Type: PT END
Name: DAS  Time: 449243621 cycles  Type: PT START
Name: DAS  Time: 449244051 cycles  Type: PT END
Name: Display  Time: 449279243 cycles  Type: FG START
Name: IdleTask  Time: 449279689 cycles  Type: FG START
Name: DAS  Time: 449283621 cycles  Type: PT START
Name: DAS  Time: 449284053 cycles  Type: PT END
Name: ADC  Time: 449287475 cycles  Type: PT START
Name: ADC  Time: 449288149 cycles  Type: PT END
Name: DAS  Time: 449323619 cycles  Type: PT START
Name: DAS  Time: 449324049 cycles  Type: PT END
Name: DAS  Time: 449363619 cycles  Type: PT START
Name: DAS  Time: 449364049 cycles  Type: PT END
Name: DAS  Time: 449403619 cycles  Type: PT START
Name: DAS  Time: 449404051 cycles  Type: PT END"""

log_re = re.compile(r"Name: (\w+)  Time: (\d+) cycles  Type: (FG START|PT START|PT END)")

events = []

for line in example.splitlines():
  match = log_re.match(line)
  if match:
    match = match.groups()
    time_ms = int(match[1])/80000.0
    events.append((match[0], time_ms, match[2]))

task_names = list(set([e[0] for e in events]))

events = [[task_names.index(e[0]), e[1], e[2]] for e in events]

fg_events = filter(lambda e: e[2].startswith("FG") or e[2] == "PT END", events)
bg_events = filter(lambda e: e[2] == "PT START", events)

stack = []
for i, e in enumerate(events):
  if e[2] == "PT START":
    stack.append(events[i-1][0])
  if e[2] == "PT END":
    events[i][0] = stack.pop()

events = sorted(fg_events+bg_events, key=lambda x: x[1])

print events

x = np.array([e[1] for e in events])
time_diffs = [j-i for i, j in zip(x[:-1], x[1:])]
time_diffs.append(time_diffs[-1])
time_diffs = np.array(time_diffs)
y = np.array([e[0] for e in events])

labels = np.array(task_names)
plt.barh(y, time_diffs, left=x, color = 'red', edgecolor = 'red', align='center', height=1)
plt.ylim(max(y)+0.5, min(y)-0.5)
plt.yticks(np.arange(y.max()+1), labels)
plt.xlim(min(x)-1, max(x)+1)
plt.xlabel("Time (ms)")
plt.ylabel("Task")
plt.show()