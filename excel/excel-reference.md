# excel-reference

## Times / Durations

Keep a running total of hours by start and end time:

`C5` = Start Time

`D5` = End Time

```visual-basic
=TEXT(D5-C5, "h:mm")
```

Summing up the total duration:

`F:F` = Duration column

```visual-basic
=ArrayFormula(text(sum(iferror(timevalue(F:F),0)), "[h]:mm"))
```

## Array Formulas

Operate on cells individually as if they were items in an array.

Press `Ctrl + Shift + Enter` after you've entered the formula.

Alternatively, add the following around the formula:

```visual-basic
=ArrayFormula(...)
```
