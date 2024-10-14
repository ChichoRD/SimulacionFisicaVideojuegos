#include <vector>
#include <cstdint>
#include <any>
#include <typeinfo>
#include <typeindex>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <cassert>
#include <functional>
#include <tuple>

#include <iostream>

#include "../objects/mass_particle.hpp"

namespace systems {
	struct particle_system {
	public:
		std::vector<std::vector<uint8_t>> particle_data;
		std::vector<std::vector<bool>> particle_data_mask;

		typedef size_t particle_id;
		typedef size_t attribute_id;
		std::unordered_map<std::type_index, attribute_id> attribute_map;

	public:
		particle_system() noexcept;
		particle_system(size_t particle_capacity, size_t attribute_capacity) noexcept;
		//~particle_system();

	public:
		typedef size_t particle_count_t;
		typedef size_t attribute_count_t;

		inline attribute_count_t attribute_count() const noexcept {
			return particle_data_mask.size();
		}

		inline particle_count_t particle_count() const {
			return attribute_count() > 0 ? particle_data_mask.at(0).size() : 0;
		}

	private:
		template <class Map, class Key>
		static typename Map::mapped_type& get_or_insert(Map& m, Key const& key, typename Map::mapped_type const& value) {
			return m.insert(typename Map::value_type(key, value)).first->second;
		}

		template <typename T>
		static constexpr size_t particle_data_index(particle_id particle)  {
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

		template <typename T>
		bool get_or_emit_attribute(attribute_id& out_attribute) const {
			auto it = attribute_map.find(typeid(T));
			if (it != attribute_map.end()) {
				out_attribute = it->second;
				return true;
			} else {
				out_attribute = particle_data.size();
				return false;
			}
		}

	public:
		template <typename T>
		constexpr bool particle_has_attribute(particle_id particle) {
			std::vector<bool> const mask = particle_data_mask.at(get_or_register_attribute<T>());
			return particle < mask.size() && mask.at(particle);
		}

		template <typename T>
		constexpr bool particle_has_attribute(particle_id particle) const {
			attribute_id attribute = 0;
			if (get_or_emit_attribute<T>(attribute)) {
				std::vector<bool> const mask = particle_data_mask.at(attribute);
				return particle < mask.size() && mask.at(particle);
			} else if (attribute < particle_data_mask.size()) {
				assert(false && "unreachable: attribute and data_mask mismatch");
				std::exit(EXIT_FAILURE);
				return false;
			} else {
				return false;
			}
		}

		template <typename T>
		T& get_particle_attribute(particle_id particle) {
			assert(particle_has_attribute<T>(particle) && "error: attempted to access non-existing attribute of particle");
			return reinterpret_cast<T&>(particle_data.at(get_or_register_attribute<T>()).at(particle_data_index<T>(particle)));
		}

		template <typename T>
		constexpr T const& get_particle_attribute(particle_id particle) const {
			attribute_id attribute = 0;
			assert(
				get_or_emit_attribute<T>(attribute)
				&& particle_has_attribute<T>(particle)
				&& "error: attempted to access non-existing attribute of particle"
			);
			return reinterpret_cast<T&>(particle_data.at(attribute).at(particle_data_index<T>(particle)));
		}


		template <typename T>
		T* get_particle_attribute_ptr(particle_id particle) {
			if (particle_has_attribute<T>(particle)) {
				return &get_particle_attribute<T>(particle);
			} else {
				return static_cast<T*>(nullptr);
			}
		}

		template <typename T>
		constexpr T const* get_particle_attribute_const_ptr(particle_id particle) const {
			if (particle_has_attribute<T>(particle)) {
				return &get_particle_attribute<T>(particle);
			} else {
				return static_cast<T const*>(nullptr);
			}
		}

		template <typename T>
		T *set_particle_attribute(particle_id particle, T&& attribute) {
			attribute_id attribute_id = get_or_register_attribute<T>();
			std::vector<bool> &mask = particle_data_mask.at(attribute_id);
			std::vector<uint8_t> &data = particle_data.at(attribute_id);

			size_t const data_index = particle_data_index<T>(particle);
			size_t const data_size = data.size();

			bool particle_in_mask = particle < mask.size();
			bool particle_in_data = data_index + sizeof(T) <= data_size;
			if (particle_in_mask != particle_in_data) {
				assert(false && "unreachable: attribute data and mask mismatch");
				std::exit(EXIT_FAILURE);
			}

			if (!particle_in_mask) {
				mask.resize(particle - mask.size() + 1);
			}
			mask.at(particle) = true;

			if (!particle_in_data) {
				size_t const missing_bytes = data_index + sizeof(T) - data_size;
				data.resize(data_size + missing_bytes);
			}

			return static_cast<T *>(std::memcpy(&data.at(data_index), &attribute, sizeof(T)));
		}

		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_particle_attribute) {
			attribute_id const attribute_id = get_or_register_attribute<T>();
			if (!particle_has_attribute<T>(particle))
				return false;

			particle_data_mask.at(attribute_id).at(particle) = false;
			out_particle_attribute = reinterpret_cast<T&>(particle_data.at(attribute_id).at(
				particle_data_index<T>(particle)
			));
			return true;
		}

	private:
		template<typename Function, typename Tuple, size_t ... I>
		auto call(Function f, Tuple t, std::index_sequence<I ...>)
		{
			return f(std::get<I>(t) ...);
		}

		template<typename Function, typename Tuple>
		auto call(Function f, Tuple t)
		{
			static constexpr auto size = std::tuple_size<Tuple>::value;
			return call(f, t, std::make_index_sequence<size>{});
		}

		template<typename ...T, size_t... I>
		auto make_references(std::tuple<T...>& t, std::index_sequence<I...>) {
			return std::tie(*std::get<I>(t)...);
		}

		template<typename ...T>
		auto make_references(std::tuple<T...>& t) {
			return make_references<T...>(t, std::make_index_sequence<sizeof...(T)>{});
		}

	public:
		template <typename ...Ts>	
		particle_count_t iter(std::function<void(Ts& ...)> const &func) {
			particle_count_t count = 0;
			for (size_t i = 0; i < particle_count(); ++i) {
				bool all = true;
				std::tuple<Ts* ...> attributes = {
					[&]() {
						Ts* ptr = get_particle_attribute_ptr<Ts>(i);
						all &= ptr != nullptr;
						return ptr;
					}()...
				};

				if (all) {
					++count;
					call(func, make_references(attributes));
				}
			}

			return count;
		}
	};
}