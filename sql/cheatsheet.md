# cheatsheet

From Claude AI.

## SQL Cheat Sheet <a href="#h-1" id="h-1"></a>

### Basic Queries <a href="#h-2" id="h-2"></a>

```sql
-- Basic SELECT
SELECT column1, column2
FROM table_name;

-- Select all columns
SELECT *
FROM table_name;

-- Select distinct values
SELECT DISTINCT column
FROM table_name;

-- Select with condition
SELECT *
FROM table_name
WHERE condition;
```

### Filtering and Sorting <a href="#h-3" id="h-3"></a>

```sql
-- WHERE clause operators
column = value     -- Equal
column > value     -- Greater than
column < value     -- Less than
column >= value    -- Greater than or equal
column <= value    -- Less than or equal
column != value    -- Not equal
column <> value    -- Not equal (alternative)

-- Multiple conditions
WHERE condition1 AND condition2
WHERE condition1 OR condition2
WHERE NOT condition
WHERE column IN (value1, value2)
WHERE column BETWEEN value1 AND value2
WHERE column LIKE pattern
WHERE column IS NULL
WHERE column IS NOT NULL

-- Pattern Matching with LIKE
WHERE column LIKE 'A%'    -- Starts with A
WHERE column LIKE '%A'    -- Ends with A
WHERE column LIKE '%A%'   -- Contains A
WHERE column LIKE '_A%'   -- Second character is A

-- Sorting
ORDER BY column ASC       -- Ascending order
ORDER BY column DESC      -- Descending order
ORDER BY col1 ASC, col2 DESC
```

### Aggregation and Grouping <a href="#h-4" id="h-4"></a>

```sql
-- Aggregate functions
SELECT COUNT(column)    -- Count rows
SELECT COUNT(DISTINCT column)
SELECT SUM(column)      -- Sum values
SELECT AVG(column)      -- Average
SELECT MAX(column)      -- Maximum value
SELECT MIN(column)      -- Minimum value

-- Grouping
SELECT column1, COUNT(*)
FROM table_name
GROUP BY column1
HAVING COUNT(*) > value;
```

### Joins <a href="#h-5" id="h-5"></a>

```sql
-- Inner Join
SELECT *
FROM table1
INNER JOIN table2
ON table1.id = table2.id;

-- Left (Outer) Join
SELECT *
FROM table1
LEFT JOIN table2
ON table1.id = table2.id;

-- Right (Outer) Join
SELECT *
FROM table1
RIGHT JOIN table2
ON table1.id = table2.id;

-- Full (Outer) Join
SELECT *
FROM table1
FULL OUTER JOIN table2
ON table1.id = table2.id;
```

### Insert, Update, Delete <a href="#h-6" id="h-6"></a>

```sql
-- Insert single row
INSERT INTO table_name (column1, column2)
VALUES (value1, value2);

-- Insert multiple rows
INSERT INTO table_name (column1, column2)
VALUES 
    (value1, value2),
    (value3, value4);

-- Update
UPDATE table_name
SET column1 = value1, column2 = value2
WHERE condition;

-- Delete
DELETE FROM table_name
WHERE condition;

-- Delete all rows
TRUNCATE TABLE table_name;
```

### Table Operations <a href="#h-7" id="h-7"></a>

```sql
-- Create table
CREATE TABLE table_name (
    column1 datatype1 constraints,
    column2 datatype2 constraints,
    ...
);

-- Add column
ALTER TABLE table_name
ADD column_name datatype;

-- Drop column
ALTER TABLE table_name
DROP COLUMN column_name;

-- Drop table
DROP TABLE table_name;
```

### Common Data Types <a href="#h-8" id="h-8"></a>

```sql
-- Numeric
INT or INTEGER       -- Whole numbers
DECIMAL(p,s)        -- Decimal numbers
FLOAT               -- Floating point
MONEY               -- Currency

-- String
CHAR(n)             -- Fixed-length
VARCHAR(n)          -- Variable-length
TEXT                -- Large text

-- Date/Time
DATE                -- YYYY-MM-DD
TIME                -- HH:MI:SS
DATETIME            -- DATE + TIME
TIMESTAMP           -- Date and time with timezone

-- Others
BOOLEAN             -- True/False
BINARY             -- Binary data
UUID               -- Unique identifier
```

### Common Constraints <a href="#h-9" id="h-9"></a>

```sql
NOT NULL            -- Cannot be null
UNIQUE              -- Must be unique
PRIMARY KEY         -- Primary key
FOREIGN KEY         -- References another table
CHECK               -- Must satisfy condition
DEFAULT value       -- Default value
```

### Useful String Functions <a href="#h-10" id="h-10"></a>

```sql
CONCAT(str1, str2)          -- Concatenate strings
UPPER(str)                  -- Convert to uppercase
LOWER(str)                  -- Convert to lowercase
LENGTH(str)                 -- String length
TRIM(str)                  -- Remove leading/trailing spaces
SUBSTRING(str, start, len)  -- Extract substring
LEFT(str, n)               -- Left n characters
RIGHT(str, n)              -- Right n characters
```

### Date Functions <a href="#h-11" id="h-11"></a>

```sql
CURRENT_DATE              -- Current date
CURRENT_TIME              -- Current time
CURRENT_TIMESTAMP         -- Current date and time
DATE_ADD(date, INTERVAL)  -- Add to date
DATE_SUB(date, INTERVAL)  -- Subtract from date
DATEDIFF(date1, date2)    -- Difference between dates
YEAR(date)                -- Extract year
MONTH(date)               -- Extract month
DAY(date)                 -- Extract day
```

