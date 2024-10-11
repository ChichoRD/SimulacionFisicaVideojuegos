#include <vector>
#include <cstdint>
#include <any>
#include <typeinfo>
#include <unordered_map>
#include <cassert>

#include "../objects/mass_particle.hpp"

namespace systems {
	struct particle_system {
	public:
		std::vector<std::vector<uint8_t>> particle_data;
		std::vector<std::vector<bool>> particle_data_mask;

		typedef size_t particle_id;
		typedef size_t attribute_id;
		std::unordered_map<std::type_info, attribute_id> attribute_map;

	public:


	private:
		template <class Map, class Key>
		static typename Map::mapped_type& get_or_insert(Map& m, Key const& key, typename Map::mapped_type const& value) {
			return m.insert(typename Map::value_type(key, value)).first->second;
		}

		template <typename T>
		static inline size_t particle_data_index(particle_id particle)  {
			return (sizeof(T) / sizeof(uint8_t)) * particle;
		}

		template <typename T>
		attribute_id get_or_register_attribute() {
			attribute_id inserted = get_or_insert(attribute_map, typeid(T), particle_data.size());
			if (inserted == particle_data.size()) {
				particle_data.push_back(std::vector<uint8_t>());
				particle_data_mask.push_back(std::vector<bool>());
			}
			return inserted;
		}

	public:

		template <typename T>
		T *set_particle_attribute(particle_id particle, T&& attribute) {
			attribute_id attribute_id = get_or_register_attribute<T>();
			std::vector<bool> mask = particle_data_mask.at(attribute_id);
			std::vector<uint8_t> data = particle_data.at(attribute_id);

			size_t data_index = particle_data_index(particle);
			size_t data_size = data.size();

			bool particle_in_mask = particle < mask.size();
			bool particle_in_data = data_index + sizeof(T) <= data_size;
			if (particle_in_mask != particle_in_data) {
				assert(false && "unreachable: attribute data and mask mismatch");
				std::exit(EXIT_FAILURE);
			}

			if (!particle_in_mask) {
				mask.resize(particle - mask.size());
			}
			mask.at(particle) = true;

			if (!particle_in_data) {
				size_t missing_bytes = data_size - data_index - sizeof(T);
				data.resize(data_size + missing_bytes);
			}
			return std::memcpy(&data.at(data_index), &attribute, sizeof(T));
		}

		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_particle_attribute) {
			attribute_id attribute_id = get_or_register_attribute<T>();
			if (particle >= particle_data_mask.at(attribute_id).size())
				return false;

			particle_data_mask.at(attribute_id).at(particle) = false;
			out_particle_attribute = particle_data.at(attribute_id).at(
				particle_data_index<T>(particle)
			);
			return true;
		}
	};
}