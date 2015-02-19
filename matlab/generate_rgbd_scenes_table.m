function generate_rgbd_scenes_table()

#instances = {
#{'bowl_2', 0.21, 0.295},
#{'bowl_3', 0.09, 0.14},
#{'bowl_4', 0.51, 0.55},
#{'cap_1', 0.545, 0.66},
#{'cap_3', 0.08, 0.08},
#{'cap_4', 0.565, 0.605},
#{'cereal_box_1', 0.435, 0.495},
#{'cereal_box_2', 0.26, 0.285},
#{'cereal_box_4', 0.76, 0.72},
#{'coffee_mug_1', 0.29, 0.42},
#{'coffee_mug_4', 0.33, 0.395},
#{'coffee_mug_5', 0.39, 0.4},
#{'coffee_mug_6', 0.025, 0.035},
#'flashlight_1', 0.535, 0.5},
#{'flashlight_2', 0.215, 0.225},
#{'flashlight_3', 0.555, 0.57},
#{'flashlight_5', 0.39, 0.445},
#{'soda_can_1', 0.235, 0.295},
#{'soda_can_3', 0.425, 0.46},
#{'soda_can_4', 0.095, 0.11},
#{'soda_can_5', 0.01, 0.015},
#{'soda_can_6', 0.7, 0.65}
#};

instances = {
{'bowl_2', 0.25, 0.08},
{'bowl_3', 0.21, 0.125},
{'bowl_4', 0.845, 0.475},
{'cap_1', 0.725, 0.11},
{'cap_3', 0.15, 0.035},
{'cap_4', 0.58, 0.17},
{'cereal_box_1', 0.5, 0.14},
{'cereal_box_2', 0.315, 0.07},
{'cereal_box_4', 0.795, 0.575},
{'coffee_mug_1', 0.38, 0.26},
{'coffee_mug_4', 0.445, 0.045},
{'coffee_mug_5', 0.56, 0.14},
{'coffee_mug_6', 0.03, 0.025},
{'flashlight_1', 0.455, 0.195},
{'flashlight_2', 0.24, 0.065},
{'flashlight_3', 0.47, 0.165},
{'flashlight_5', 0.4, 0.23},
{'soda_can_1', 0.295, 0.195},
{'soda_can_3', 0.525, 0.375},
{'soda_can_4', 0.165, 0.07},
{'soda_can_5', 0.035, 0.02},
{'soda_can_6', 0.615, 0.45}
};

categories = {};
field = 3;

for i = 1:length(instances)
    category = instances{i}{1}(1:end-2);
    found = false;
    for j = 1:length(categories)
        if strcmp(categories{j}{1}, category)
            categories{j}{2} = categories{j}{2} + instances{i}{3};
            categories{j}{3} = categories{j}{field} + 1;
            found = true;
            break;
        end
    end
    if ~found
        categories{length(categories)+1} = {category, instances{i}{field}, 1};
    end
end

category_names = '';
category_values = [];
for j = 1:length(categories)
    category_names = [category_names ' ' categories{j}{1}];
    categories{j}{2} = categories{j}{2}/categories{j}{3};
    category_values = [category_values categories{j}{2}];
end

category_names
category_values
mean(category_values)

end
