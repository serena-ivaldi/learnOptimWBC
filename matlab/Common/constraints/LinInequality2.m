% inequality of the kind  x > k    ------>  - x + k < 0
function violation = LinInequality2(input,constraints_values)
    violation = -input + constraints_values;
end