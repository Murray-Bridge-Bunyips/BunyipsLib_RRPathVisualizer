// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bunyipslib.external.units.collections;

import java.util.Arrays;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.stream.LongStream;

/**
 * A read-only set of unique primitive {@code long} values.
 */
public class ReadOnlyPrimitiveLongSet implements Iterable<Long> {
    private final long[] values;

    /**
     * Creates a new set from the given values. These values do not have to be unique.
     *
     * @param values the values that belong to the set.
     */
    public ReadOnlyPrimitiveLongSet(long... values) {
        // Initial size is the upper limit
        long[] uniqueValues = new long[values.length];
        int numUniqueValues = 0;
        boolean seenZero = false;

        // Copy the set of unique values to our array using indexed for-loops to avoid allocations
        copyLoop:
        for (long value : values) {
            if (value == 0 && !seenZero) {
                // Special case to support zero
                seenZero = true;
            } else {
                for (long uniqueValue : uniqueValues) {
                    if (uniqueValue == value) {
                        continue copyLoop;
                    }
                }
            }
            uniqueValues[numUniqueValues] = value;
            numUniqueValues++;
        }

        if (numUniqueValues == values.length) {
            // All input values were unique, no need to truncate
            this.values = uniqueValues;
        } else {
            // Truncate the array to remove trailing empty space
            this.values = Arrays.copyOf(uniqueValues, numUniqueValues);
        }
    }

    /**
     * Checks if the set contains a particular value.
     *
     * @param value the value to check for
     * @return true if the value is in the set, false if not
     */
    public boolean contains(long value) {
        for (long mValue : values) {
            if (mValue == value) {
                return true;
            }
        }
        return false;
    }

    /**
     * Retrieves the number of elements in the set.
     *
     * @return the number of elements in the set
     */
    public int size() {
        return values.length;
    }

    /**
     * Checks if the set is empty, i.e. contains no values.
     *
     * @return true if there are no values in the set, false otherwise.
     */
    public boolean isEmpty() {
        return size() == 0;
    }

    /**
     * Creates a stream of primitive long values for the set.
     *
     * @return a sequential Stream over the elements in this collection
     * @see Set#stream()
     */
    public LongStream stream() {
        return Arrays.stream(values);
    }

    /**
     * Creates a new array that contains all of the values in the set.
     *
     * @return an array containing all the values in the set
     */
    public long[] toArray() {
        return Arrays.copyOf(values, values.length);
    }

    @Override
    public Iterator<Long> iterator() {
        return new Iterator<Long>() {
            private int index = 0;

            @Override
            public boolean hasNext() {
                return index < size();
            }

            @Override
            public Long next() {
                if (!hasNext()) {
                    throw new NoSuchElementException("No more elements in the set.");
                }

                long l = values[index];
                index++;
                return l;
            }
        };
    }
}
