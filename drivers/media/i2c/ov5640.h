#ifndef OV5640_H
#define OV5640_H

/**
 * struct OV5640_platform_data - OV5640 driver platform data
 * @link_frequency: target pixel clock frequency
 */
struct ov5640_platform_data {
	s64 link_frequency;
};

#endif /* OV5640_H */

