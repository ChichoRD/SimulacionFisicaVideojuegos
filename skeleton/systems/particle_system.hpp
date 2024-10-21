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

namespace systems {
	typedef size_t particle_id;
	typedef size_t attribute_id;
	typedef size_t particle_count_t;
	typedef size_t attribute_count_t;

	struct attribute_storage {
	public:
		enum storage_type {
			vector,
			unit,
			shared
		};

	public:
		std::type_info const &type_info;
		size_t const type_size;

		std::vector<bool> mask;
		std::vector<uint8_t> data;
		storage_type storage_variant;

	public:
		attribute_storage()
			: type_info(typeid(bool)), type_size(), mask(), data(), storage_variant(storage_type::unit) { }

		attribute_storage(size_t particle_capacity, storage_type storage_type, std::type_info const &type_info, size_t type_size)
			: type_info(type_info), type_size(type_size), mask(), data(), storage_variant(storage_type) {
			switch (storage_type) {
			case storage_type::vector: {
				mask.reserve(particle_capacity);
				data.reserve(particle_capacity * type_size);
				break;
			}

			case storage_type::unit: {
				assert(type_info == typeid(bool) && "error: invalid storage type for unit storage");
				mask.reserve(particle_capacity);
				data.~vector();
			}

			case storage_type::shared: {
				mask.resize(1, false);
				data.resize(type_size);
				break;
			}

			default: {
				assert(false && "unreachable: invalid storage type");
				std::exit(EXIT_FAILURE);
				break;
			}
			}
		}

		template <typename T>
		static attribute_storage create_vector_storage(size_t particle_capacity) {
			return attribute_storage(particle_capacity, storage_type::vector, typeid(T), sizeof(T));
		}

		static attribute_storage create_unit_storage(size_t particle_capacity) {
			return attribute_storage(particle_capacity, storage_type::unit, typeid(bool), sizeof(bool));
		}

		template <typename T>
		static attribute_storage create_shared_storage(size_t particle_capacity) {
			return attribute_storage(particle_capacity, storage_type::shared, typeid(T), sizeof(T));
		}

	public:
		inline particle_count_t particle_count() const {
			return mask.size();
		}

		template <typename T>
		static constexpr size_t particle_data_index(particle_id particle)  {
			return (sizeof(T) / sizeof(uint8_t)) * particle;
		}
	
	public:
		template <typename T>
		constexpr bool particle_has_attribute(particle_id particle) const {
			return particle < mask.size() && mask.at(particle);
		}

		template <typename T>
		T& get_particle_attribute(particle_id particle) {
			assert(typeid(T) == type_info && "error: attempted to access attribute of wrong type");
			assert(particle_has_attribute<T>(particle) && "error: attempted to access non-existing attribute of particle");

			switch (storage_variant) {
			case vector:
				return reinterpret_cast<T&>(data.at(particle_data_index<T>(particle)));
			case unit: {
				return reinterpret_cast<T&>(mask.at(particle));
			}
			case shared: {
				if (data.size() != type_size) {
					assert(false && "unreachable: data size mismatch, given shared storage");
					std::exit(EXIT_FAILURE);
				}
				return reinterpret_cast<T&>(data.at(0));
			}
			default: {
				assert(false && "unreachable: invalid storage type");
				std::exit(EXIT_FAILURE);
			}
			}
		}

		template <typename T>
		constexpr T const& get_particle_attribute(particle_id particle) const {
			return get_particle_attribute<T>(particle);
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
			assert(typeid(T) == type_info && "error: attempted to access attribute of wrong type");

			switch (storage_variant) {
			case vector: {
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
			case unit: {
				mask.at(particle) = true;
				return reinterpret_cast<T *>(&mask.at(particle));
			}
			case shared: {
				if (mask.size() != 1 || data.size() != type_size) {
					assert(false && "unreachable: mask size mismatch, given shared storage");
					std::exit(EXIT_FAILURE);	
					return nullptr;
				}
				mask.at(0) = true;
				reinterpret_cast<T&>(data.at(0)) = attribute;

				return reinterpret_cast<T*>(&data.at(0));
			}
			default: {
				assert(false && "unreachable: invalid storage type");
				std::exit(EXIT_FAILURE);
				return nullptr;
			}
			}
		}

		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_particle_attribute) {
			assert(typeid(T) == type_info && "error: attempted to access attribute of wrong type");

			if (!particle_has_attribute<T>(particle)) {
				return false;
			}

			switch (storage_variant) {
			case vector: {
				out_particle_attribute = get_particle_attribute<T>(particle);
				mask.at(particle) = false;
				return true;
			}
			case unit: {
				mask.at(particle) = false;
				return true;
			}
			case shared: {
				if (mask.size() != 1 || data.size() != type_size) {
					assert(false && "unreachable: mask size mismatch, given shared storage");
					std::exit(EXIT_FAILURE);	
					return nullptr;
				}

				mask.at(0) = false;
				out_particle_attribute = get_particle_attribute<T>(particle);
				return true;
			}
			}
		}
	};

	struct particle_system {
	public:
		std::vector<attribute_storage> particles;
		std::unordered_map<std::type_index, attribute_id> attribute_map;

	public:
		particle_system() noexcept;
		particle_system(size_t attribute_capacity) noexcept;

	public:
		inline attribute_count_t attribute_count() const noexcept {
			return particles.size();
		}

		inline particle_count_t particle_count() const {
			return attribute_count() > 0 ? particles.at(0).particle_count() : 0;
		}

	private:
		// template <class Map, class Key>
		// static typename Map::mapped_type& get_or_insert(Map& m, Key const& key, typename Map::mapped_type const& value) {
		// 	return m.insert(typename Map::value_type(key, value)).first->second;
		// }

		template <typename T>
		bool get_or_emit_attribute(attribute_id& out_attribute) const {
			auto it = attribute_map.find(typeid(T));
			if (it != attribute_map.end()) {
				out_attribute = it->second;
				return true;
			} else {
				out_attribute = attribute_count();
				return false;
			}
		}
	
	public:
		template <typename T>
		bool particle_has_attribute(particle_id particle) const {
			attribute_id attribute = 0;
			if (!get_or_emit_attribute<T>(attribute)) {
				return false;
			}
			return particles.at(attribute).particle_has_attribute<T>(particle);
		}


		template <typename T>
		T &get_particle_attribute(particle_id particle) {
			attribute_id attribute = 0;
			assert(get_or_emit_attribute<T>(attribute) && "error: attribute not found");
			return particles.at(attribute).get_particle_attribute<T>(particle);
		}

		template <typename T>
		T const &get_particle_attribute(particle_id particle) const {
			attribute_id attribute = 0;
			assert(get_or_emit_attribute<T>(attribute) && "error: attribute not found");
			return particles.at(attribute).get_particle_attribute<T>(particle);
		}

		template <typename T>
		T *get_particle_attribute_ptr(particle_id particle) {
			attribute_id attribute = 0;
			if (get_or_emit_attribute<T>(attribute)) {
				return particles.at(attribute).get_particle_attribute_ptr<T>(particle);
			} else {
				return nullptr;
			}
		}

		template <typename T>
		T const *get_particle_attribute_const_ptr(particle_id particle) const {
			attribute_id attribute = 0;
			if (get_or_emit_attribute<T>(attribute)) {
				return particles.at(attribute).get_particle_attribute_const_ptr<T>(particle);
			} else {
				return nullptr;
			}
		}


		template <typename T>
		T &set_particle_attribute(particle_id particle, T attribute) {
			attribute_id attribute_id = 0;
			if (get_or_emit_attribute<T>(attribute_id)) {
				return *particles.at(attribute_id).set_particle_attribute<T>(particle, std::move(attribute));
			} else {
				attribute_map.insert({ typeid(T), attribute_id });
				particles.push_back(attribute_storage::create_vector_storage<T>(particle_count()));
				return *particles.at(attribute_id).set_particle_attribute<T>(particle, std::move(attribute));
			}
		}

		
		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_attribute) {
			attribute_id attribute_id = 0;
			if (!get_or_emit_attribute<T>(attribute_id)) {
				return false;
			}

			return particles.at(attribute_id).remove_particle_attribute<T>(particle, out_attribute);
		}

	private:
		template <
			typename T,
			typename ...Attributes,
			typename Tuple = T::particle_deconstruct
		>
		std::tuple<Attributes& ...> set_particle_attributes_deconstruct_impl(
			particle_id particle,
			T const &attributes,
			std::tuple<Attributes ...>
		) {
			return { set_particle_attribute<Attributes>(
				particle,
				std::move(std::get<Attributes>(attributes.deconstruct()))
			)... };
		}

		template <
			typename T,
			typename ...Attributes,
			typename = typename std::enable_if_t<std::is_constructible_v<T, Attributes...>>
		>
		T get_particle_attributes_construct_impl(particle_id particle, std::tuple<Attributes ...>) {
			return T(get_particle_attribute<Attributes>(particle)...);
		}

	public:
		template <
			typename T,
			typename ...Attributes,
			typename Tuple = T::particle_deconstruct
		>
		auto set_particle_attributes_deconstruct(particle_id particle, T const &attributes) noexcept {
			return set_particle_attributes_deconstruct_impl<T>(
				particle,
				attributes, 
				Tuple{}
			);
		}

		template <
			typename T,
			typename Tuple = T::particle_deconstruct
		>
		T get_particle_attributes_construct(particle_id particle) {
			return get_particle_attributes_construct_impl<T>(particle, Tuple{});
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
		template <
			typename ...Ts,
			typename F = std::function<void(Ts& ...)>
		>	
		particle_count_t iter(F const &func) {
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