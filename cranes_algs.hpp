///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes {

// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) {

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    // Compute maximum path length, and check that it is legal.
    const size_t max_steps = setting.rows() + setting.columns() - 2;

    std::cout << "max_steps:" << max_steps << " ";

    assert(max_steps < 64);

    path best(setting);

    // Iterate over all possible step combinations up to max_steps
for (size_t steps = 0; steps <= max_steps; steps++) {
    // Iterate over all possible bit combinations of length `steps`
    for (int i = 0; i < (1 << steps); i++) {
        // Keep track of the current best path
        size_t max_cranes = best.total_cranes();
        // Create a new path starting from the initial setting
        path valid_path(setting);

        // For each step, determine whether to move SOUTH or EAST based on the current bit in `i`
        for (size_t j = 0; j <= steps - 1; j++){
            step_direction step_direction;

            if (((i >> j) & 1) == 0){
                step_direction = STEP_DIRECTION_SOUTH;
            } else {
                step_direction = STEP_DIRECTION_EAST;
            }

            // If the step is valid, add it to the path
            if (valid_path.is_step_valid(step_direction)){
                valid_path.add_step(step_direction);
            } else {
                // If the step is not valid, break out of the loop and move on to the next bit combination
                break;
            }
        }

        // If the new path has more cranes than the current best path, update the current best path
        if (valid_path.total_cranes() > max_cranes){
            max_cranes = valid_path.total_cranes();
            best = valid_path;
        }
    }
}

// Return the best path found
return best;


}

// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
//path crane_unloading_dyn_prog(const grid& setting) {
path crane_unloading_dyn_prog(const grid& setting) {

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    using cell_type = std::optional<path>;

    std::vector<std::vector<cell_type> > A(setting.rows(),
                                           std::vector<cell_type>(setting.columns()));

    A[0][0] = path(setting);
    assert(A[0][0].has_value());

    // Iterate over each cell in the grid
for (coordinate r = 0; r < setting.rows(); ++r) {
    for (coordinate c = 0; c < setting.columns(); ++c) {

        // Reset cell if it's a building
        if (setting.get(r, c) == CELL_BUILDING){
            A[r][c].reset();
            continue;
        }

        // Initialize variables for checking if there's a path from above or left
        cell_type from_above = std::nullopt;
        cell_type from_left = std::nullopt;
        bool is_from_above = false;
        bool is_from_left = false;

        // Check if there's a path from above
        if (r>0 && A[r-1][c].has_value()){
            is_from_above = true;
        }

        // Check if there's a path from left
        if (c>0 && A[r][c-1].has_value()){
            is_from_left = true;
        }

        // If both paths exist, add step in the direction with more cranes
        if (is_from_above && is_from_left){
            if (A[r-1][c]->total_cranes() > A[r][c-1]->total_cranes()) {
                if( A[r-1][c]->is_step_valid(STEP_DIRECTION_SOUTH)){
                    from_above = A[r-1][c];
                    from_above->add_step(STEP_DIRECTION_SOUTH);
                    A[r][c] = from_above;
                }
            } else if(A[r][c-1]->is_step_valid(STEP_DIRECTION_EAST)){
                from_left = A[r][c-1];
                from_left->add_step(STEP_DIRECTION_EAST);
                A[r][c] = from_left;
            }
        }
        // If only the path from left exists, continue in that direction
        else if(is_from_left) {
            if(A[r][c-1]->is_step_valid(STEP_DIRECTION_EAST)){
                from_left = A[r][c-1];
                from_left->add_step(STEP_DIRECTION_EAST);
                A[r][c] = from_left;
            }
        }
        // If only the path from above exists, continue in that direction
        else if(is_from_above) {
            if( A[r-1][c]->is_step_valid(STEP_DIRECTION_SOUTH)){
                from_above = A[r-1][c];
                from_above->add_step(STEP_DIRECTION_SOUTH);
                A[r][c] = from_above;
            }
        }
    }
}

// Find the path with the most cranes after the previous iterations
cell_type* best = &(A[0][0]);
for (coordinate r = 0; r < setting.rows(); ++r) {
    for (coordinate c = 0; c < setting.columns(); ++c) {
        if (A[r][c].has_value()){
            if(A[r][c]->total_cranes() > (*best)->total_cranes()) {
                best = &(A[r][c]);
            }
        }
    }
}


    assert(best->has_value());
    // std::cout << "total cranes" << (**best).total_cranes() << std::endl;

    return **best;
}

}
