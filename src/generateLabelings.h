#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <iostream>

// Function to check if a list is cyclically sorted
bool is_cyclic_sorted(std::vector<int> lst)
{
    int n = lst.size();
    int break_asc = 0;
    int break_desc = 0;
    // Find the "wrap around" point for ascending order and descending order
    for (int i = 0; i < n; i++)
    {
        if (lst[i] > lst[(i + 1) % n])
        {
            break_asc = i;
        }
        if (lst[i] < lst[(i + 1) % n])
        {
            break_desc = i;
        }
    }
    std::vector<int> lst_copy = lst;
    // Rotate the list to start at the "wrap around" point for ascending order
    std::rotate(lst.begin(), lst.begin() + break_asc + 1, lst.end());
    // Check if the list is now sorted
    if (std::is_sorted(lst.begin(), lst.end()))
    {
        return true;
    }
    // Rotate the list to start at the "wrap around" point for descending order
    std::rotate(lst_copy.begin(), lst_copy.begin() + break_desc + 1, lst_copy.end());

    // Check if the list is now sorted in reverse order
    if (std::is_sorted(lst_copy.begin(), lst_copy.end(), std::greater<int>()))
    {
        return true;
    }
    return false;
}

// Function to generate permutations
void permute(std::vector<int> &elements, int l, int r, std::vector<std::vector<int>> &permutations)
{
    if (l == r)
    {
        permutations.push_back(elements);
    }
    else
    {
        for (int i = l; i <= r; i++)
        {
            std::swap(elements[l], elements[i]);
            permute(elements, l + 1, r, permutations);
            std::swap(elements[l], elements[i]); // backtrack
        }
    }
}

// Function to generate combinations
std::vector<std::vector<int>> combine(const std::vector<int> &elements, int r)
{
    std::vector<bool> mask(elements.size() - r, false); // K leading 0's
    mask.resize(elements.size(), true);                 // N-K trailing 1's

    std::vector<std::vector<int>> combinations;

    // Print permutations that are pointed to by 1's.
    do
    {
        std::vector<int> combination;
        for (int i = 0; i < elements.size(); ++i)
        { // Loop through mask vector
            if (mask[i])
            {
                combination.push_back(elements[i]);
            }
        }
        if (combination.size() != 0)
        {
            combinations.push_back(combination);
            ;
        }
    } while (std::next_permutation(mask.begin(), mask.end()));

    return combinations;
}

bool valueExistsInMap(const std::unordered_map<int, int> &map, int value)
{
    for (const auto &pair : map)
    {
        if (pair.second == value)
        {
            return true;
        }
    }
    return false;
}

std::vector<std::vector<std::pair<int, int>>> generateLabelings(std::vector<int> labels, std::vector<std::pair<int, int>> fixed_labels, std::vector<int> markers)
{
    std::unordered_map<int, int> fixed_labels_dict;
    for (const auto &p : fixed_labels)
    {
        fixed_labels_dict[p.first] = p.second;
    }

    std::vector<int> remaining_labels;
    for (const auto &label : labels)
    {
        if (!valueExistsInMap(fixed_labels_dict, label))
        {
            remaining_labels.push_back(label);
        }
    }

    std::vector<int> remaining_markers;
    for (const auto &marker : markers)
    {
        if (fixed_labels_dict.count(marker) == 0)
        {
            remaining_markers.push_back(marker);
        }
    }

    std::vector<std::vector<std::pair<int, int>>> results;

    // print size of remaining labels and remaining markers
    std::cout << "Remaining labels: " << remaining_labels.size() << std::endl;
    std::cout << "Remaining markers: " << remaining_markers.size() << std::endl;

    // Generate all possible combinations of remaining labels
    auto combinations = combine(remaining_labels, remaining_markers.size());
    // print combinations
    std::cout << "Combinations: " << combinations.size() << std::endl;
    for (auto combination : combinations)
    {
        for (auto label : combination)
        {
            std::cout << label << " ";
        }
        std::cout << std::endl;
    }
    if (combinations.size() == 0)
    {
        std::vector<std::pair<int, int>> combined = fixed_labels;
        results.push_back(combined);
        return results;
    }

    for (auto combination : combinations)
    {
        // Generate all permutations of each combination
        std::vector<std::vector<int>> permutations;
        permute(combination, 0, combination.size() - 1, permutations);

        std::cout << "combination: " << std::endl;
        for (const auto label : combination)
        {
            std::cout << label << " " ;
            
        }
        std::cout << std::endl;
        for (const auto &perm : permutations)
        {
            // Zip together the remaining markers and the current permutation of labels
            std::vector<std::pair<int, int>> temp_labels;
            for (int i = 0; i < remaining_markers.size(); i++)
            {
                temp_labels.push_back(std::make_pair(remaining_markers[i], perm[i]));
            }
            // Combine the fixed and temporary labels
            std::vector<std::pair<int, int>> combined = fixed_labels;
            combined.insert(combined.end(), temp_labels.begin(), temp_labels.end());

            // Sort the combined labels by the marker order
            std::sort(combined.begin(), combined.end(), [&markers](const std::pair<int, int> &a, const std::pair<int, int> &b)
                      { return std::find(markers.begin(), markers.end(), a.first) < std::find(markers.begin(), markers.end(), b.first); });

            // Create label order based on sorted marker order
            std::vector<int> label_order;
            for (const auto& pair : combined) {
                label_order.push_back(pair.second);
            }
            // Check if the sorted labels match the original label order (with wrap-around)
            if (is_cyclic_sorted(label_order))
            {
                // std::cout << "Label order is cyclically sorted" << std::endl;
                std::cout << "Cyclic label order:" << std::endl;
                for (const auto &label : label_order)
                {
                    std::cout << label << " ";
                }
                std::cout << std::endl;
                results.push_back(combined);
            }
        }
    }

    std::cout << "Results: " << results.size() << std::endl;

    return results;
}

// int main() {
//     // std::vector<int> labels = { 0, 1, 2, 3, 4, 5 };
//     std::vector<int> labels = { 0, 1, 2, 3};
//     std::vector<std::pair<int, int>> fixed_labels = { std::make_pair(1, 2)};
//     std::vector<int> markers = { 0,1,2 , 3};

//     auto results = generateLabelings(labels, fixed_labels, markers);

//     for (const auto& result : results) {
//         for (const auto& pair : result) {
//             std::cout << pair.second << " ";
//         }
//         std::cout << std::endl;
//     }

//     return 0;
// }
